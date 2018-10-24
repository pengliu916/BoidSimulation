//
//  Renderer.swift
//  BoidSimulation Shared
//
//  Created by Peng Liu on 5/13/18.
//  Copyright © 2018 PLStudio. All rights reserved.
//

// Our platform independent renderer class

import Metal
import MetalKit
import simd

// The 256 byte aligned size of our uniform structure
let alignedUniformsSize = (MemoryLayout<Uniforms>.size + 0xFF) & ~0xFF
let alignedSimCBSize = (MemoryLayout<SimCB>.size + 0xFF) & ~0xFF

let maxBuffersInFlight = 3

let fishMeshVertex: [Float] = [ -0.3,  0.0, -0.3,
                                 1.0,  0.0,  0.0,
                                -0.3,  0.0, -0.3,
                                -1.0,  1.0,  0.0,
                                -0.3,  0.0,  0.3,
                                 1.0,  0.0,  0.0,
                                -0.3,  0.0,  0.3,
                                -1.0, -1.0,  0.0,
                                -0.3,  0.0, -0.3,
                                 1.0,  0.0,  0.0]
enum RendererError: Error {
    case badVertexDescriptor
}

class Renderer: NSObject, MTKViewDelegate {
    #if os(macOS)
    let instanceCnt: Int = Int(BLOCK_SIZE * 200)
    #else
    let instanceCnt: Int = Int(BLOCK_SIZE * 60)
    #endif
    
    var resetBoids: Bool = false
    
    var pos_vel_Buffers: [MTLBuffer] = [MTLBuffer]()
    var pos_vel_bufIn: MTLBuffer!
    var pos_vel_bufOut: MTLBuffer!
    let pos_center: vector_float3 = vector_float3(arrayLiteral: 0.0, 0.0, 0.0)
    let xyz_expand: vector_float3 = vector_float3(arrayLiteral: 60.0, 30.0, 60.0)
    var cptPSO: MTLComputePipelineState!
    var simCB: UnsafeMutablePointer<SimCB>!
    var dynamicSimCB: MTLBuffer!
    var simCBOffset = 0
    var vertexBuf: MTLBuffer!
    
    public let device: MTLDevice
    let commandQueue: MTLCommandQueue
    var dynamicUniformBuffer: MTLBuffer
    var gfxPSO: MTLRenderPipelineState!
    var depthState: MTLDepthStencilState!
    
    let inFlightSemaphore = DispatchSemaphore(value: maxBuffersInFlight)
    
    var uniformBufferOffset = 0
    
    var uniformBufferIndex = 0
    
    var uniforms: UnsafeMutablePointer<Uniforms>
    
    var projectionMatrix: matrix_float4x4 = matrix_float4x4()
    
    var rotation: Float = 0
    
    func RandAbs1() -> Float {return Float(drand48()) * 2.0 - 1.0}
    
    func CreateResources(_ device: MTLDevice, _ metalKitView: MTKView) {
        
        // Create vel-pos buffer
        buildBuffer()
        
        // Create simulation constant buffers
        let bufferSize = alignedSimCBSize * maxBuffersInFlight
        dynamicSimCB = device.makeBuffer(length: bufferSize, options: MTLResourceOptions.storageModeShared)!
        simCB = UnsafeMutableRawPointer(dynamicSimCB.contents()).bindMemory(to: SimCB.self, capacity: 1)
        
        // Create vertex resource
        vertexBuf = device.makeBuffer(bytes: fishMeshVertex, length: MemoryLayout<Float>.size * 30, options: MTLResourceOptions.storageModeShared)
        
        // Create graphics pipeline obj
        let mtlVertDesc = Renderer.buildMetalVertexDescriptor()
        do {
            gfxPSO = try Renderer.buildRenderPipelineWithDevice(device: device,
                                                                metalKitView: metalKitView,
                                                                mtlVertexDescriptor: mtlVertDesc)
        } catch {
            print("Unable to compile render pipeline state.  Error info: \(error)")
            return
        }
        
        let depthStateDesciptor = MTLDepthStencilDescriptor()
        depthStateDesciptor.depthCompareFunction = MTLCompareFunction.less
        depthStateDesciptor.isDepthWriteEnabled = true
        guard let state = device.makeDepthStencilState(descriptor:depthStateDesciptor) else { return }
        depthState = state
        
        
        let library = device.makeDefaultLibrary()
        let csShader = library?.makeFunction(name: "csSimulation")
        do {
            cptPSO = try device.makeComputePipelineState(function: csShader!)
        } catch {
            print("Unable to compile compute pipeline state. Error info: \(error)")
        }
    }
    
    init?(metalKitView: MTKView) {
        self.device = metalKitView.device!
        guard let queue = self.device.makeCommandQueue() else { return nil }
        self.commandQueue = queue
        
        let uniformBufferSize = alignedUniformsSize * maxBuffersInFlight
        
        guard let buffer = self.device.makeBuffer(length:uniformBufferSize, options:[MTLResourceOptions.storageModeShared]) else { return nil }
        dynamicUniformBuffer = buffer
        
        self.dynamicUniformBuffer.label = "UniformBuffer"
        
        uniforms = UnsafeMutableRawPointer(dynamicUniformBuffer.contents()).bindMemory(to:Uniforms.self, capacity:1)
        
        metalKitView.depthStencilPixelFormat = MTLPixelFormat.depth32Float_stencil8
        metalKitView.colorPixelFormat = MTLPixelFormat.bgra8Unorm_srgb
        metalKitView.sampleCount = 1
        
        
        super.init()
        
        CreateResources(device, metalKitView)
        
        print("\(cptPSO.maxTotalThreadsPerThreadgroup)")
        
    }
    
    func buildBuffer() {
        // Create pos vel buffers
        let bufferSize = MemoryLayout<Float>.size * 8 * instanceCnt
        #if os(macOS)
        let option = MTLResourceOptions.storageModeManaged
        #elseif os(iOS)
        let option = MTLResourceOptions.storageModeShared
        #endif
        pos_vel_Buffers.append(device.makeBuffer(length: bufferSize, options: option)!)
        pos_vel_Buffers.append(device.makeBuffer(length: bufferSize, options: option)!)
        initPosVel()
    }
    
    func initPosVel() {
        let bufferSize = MemoryLayout<Float>.size * 8 * instanceCnt
        let pos_vel = pos_vel_Buffers[0].contents().bindMemory(to: Float.self, capacity: bufferSize)
        
        let clusterScale: Float = 0.2
        let velFactor: Float = 1.0
        
        //        for i in 0..<instanceCnt {
        //            pos_vel[i * 8 + 0] = RandAbs1() * xyz_expand.x * clusterScale + pos_center.x
        //            pos_vel[i * 8 + 1] = RandAbs1() * xyz_expand.y * clusterScale + pos_center.y
        //            pos_vel[i * 8 + 2] = RandAbs1() * xyz_expand.z * clusterScale + pos_center.z
        //            pos_vel[i * 8 + 4] = RandAbs1() * velFactor
        //            pos_vel[i * 8 + 5] = RandAbs1() * velFactor
        //            pos_vel[i * 8 + 6] = RandAbs1() * velFactor
        //        }
        for i in 0..<instanceCnt {
            let z = Float(i / 100) / 9.0 - 0.5
            let y = Float(i % 100 / 10) / 9.0 - 0.5
            let x = Float(i % 10) / 9.0 - 0.5
            pos_vel[i * 8 + 0] = x * xyz_expand.x * clusterScale + pos_center.x
            pos_vel[i * 8 + 1] = y * xyz_expand.y * clusterScale + pos_center.y
            pos_vel[i * 8 + 2] = z * xyz_expand.z * clusterScale + pos_center.z
            pos_vel[i * 8 + 4] = RandAbs1() * velFactor
            pos_vel[i * 8 + 5] = RandAbs1() * velFactor
            pos_vel[i * 8 + 6] = RandAbs1() * velFactor
        }
        #if os(macOS)
        pos_vel_Buffers[0].didModifyRange(0..<bufferSize)
        #endif
        pos_vel_bufIn = pos_vel_Buffers[0]
        pos_vel_bufOut = pos_vel_Buffers[1]
    }
    
    class func buildMetalVertexDescriptor() -> MTLVertexDescriptor {
        // Creete a Metal vertex descriptor specifying how vertices will by laid out for input into our render
        //   pipeline and how we'll layout our Model IO vertices
        
        let mtlVertexDescriptor = MTLVertexDescriptor()
        
        mtlVertexDescriptor.attributes[0].format = MTLVertexFormat.float3
        mtlVertexDescriptor.attributes[0].offset = 0
        mtlVertexDescriptor.attributes[0].bufferIndex = 0
        
        mtlVertexDescriptor.layouts[0].stride = 12
        mtlVertexDescriptor.layouts[0].stepRate = 1
        mtlVertexDescriptor.layouts[0].stepFunction = MTLVertexStepFunction.perVertex
        
        return mtlVertexDescriptor
    }
    
    class func buildRenderPipelineWithDevice(device: MTLDevice,
                                             metalKitView: MTKView,
                                             mtlVertexDescriptor: MTLVertexDescriptor) throws -> MTLRenderPipelineState {
        /// Build a render state pipeline object
        
        let library = device.makeDefaultLibrary()
        
        let vertexFunction = library?.makeFunction(name: "vsRender")
        let fragmentFunction = library?.makeFunction(name: "fsRender")
        
        let pipelineDescriptor = MTLRenderPipelineDescriptor()
        pipelineDescriptor.label = "RenderPipeline"
        pipelineDescriptor.sampleCount = metalKitView.sampleCount
        pipelineDescriptor.vertexFunction = vertexFunction
        pipelineDescriptor.fragmentFunction = fragmentFunction
        pipelineDescriptor.vertexDescriptor = mtlVertexDescriptor
        
        pipelineDescriptor.colorAttachments[0].pixelFormat = metalKitView.colorPixelFormat
        pipelineDescriptor.depthAttachmentPixelFormat = metalKitView.depthStencilPixelFormat
        pipelineDescriptor.stencilAttachmentPixelFormat = metalKitView.depthStencilPixelFormat
        
        return try device.makeRenderPipelineState(descriptor: pipelineDescriptor)
    }
    
    private func updateDynamicBufferState() {
        /// Update the state of our uniform buffers before rendering
        
        uniformBufferIndex = (uniformBufferIndex + 1) % maxBuffersInFlight
        
        uniformBufferOffset = alignedUniformsSize * uniformBufferIndex
        
        uniforms = UnsafeMutableRawPointer(dynamicUniformBuffer.contents() + uniformBufferOffset).bindMemory(to:Uniforms.self, capacity:1)
        
        simCBOffset = alignedSimCBSize * uniformBufferIndex
        simCB = UnsafeMutableRawPointer(dynamicSimCB.contents() + simCBOffset).bindMemory(to: SimCB.self, capacity: 1)
    }
    
    private func updateGameState() {
        /// Update any game state before rendering
        
        uniforms[0].projectionMatrix = projectionMatrix
        
        let rotationAxis = float3(1, 1, 0)
        let modelMatrix = matrix4x4_rotation(radians: rotation, axis: rotationAxis)
        let viewMatrix = matrix4x4_translation(0.0, 0.0, -50.0)
        uniforms[0].modelViewMatrix = simd_mul(viewMatrix, modelMatrix)
        
        simCB[0].uInstanceCnt = UInt32(instanceCnt)
        simCB[0].fAvoidanceFactor = 8.0
        simCB[0].fSeperationFactor = 0.4
        simCB[0].fCohesionFactor = 15.0
        simCB[0].fAlignmentFactor = 12.0
        simCB[0].fSeekingFactor = 0.2
        simCB[0].f3SeekSourcePos = vector_float3(0.0, 0.0, 0.0)
        simCB[0].fFleeFactor = 0.0
        simCB[0].f3FleeSourcePos = vector_float3(0.0, 0.0, 0.0)
        simCB[0].fMaxForce = 200.0
        simCB[0].f3CenterPos = vector_float3(0.0, 0.0, 0.0)
        simCB[0].fMaxSpeed = 20.0
        simCB[0].f3xyzExpand = vector_float3(60.0, 30.0, 60.0)
        simCB[0].fMinSpeed = 2.5
        simCB[0].fVisionDist = 3.5
        simCB[0].fVisionAngleCos = -0.6
        simCB[0].fDeltaT = 0.032
        simCB[0].fFishSize = 0.3
    }
    
    func draw(in view: MTKView) {
        /// Per frame updates hare
        
        _ = inFlightSemaphore.wait(timeout: DispatchTime.distantFuture)
        
        if let commandBuffer = commandQueue.makeCommandBuffer() {
            
            let semaphore = inFlightSemaphore
            commandBuffer.addCompletedHandler { (_ commandBuffer)-> Swift.Void in
                semaphore.signal()
            }
            
            self.updateDynamicBufferState()
            
            self.updateGameState()
            
            let groupsize = MTLSizeMake(Int(BLOCK_SIZE), 1, 1)
            let numgroups = MTLSizeMake(instanceCnt/Int(BLOCK_SIZE), 1, 1)
            let cptEncoder = commandBuffer.makeComputeCommandEncoder()!
            cptEncoder.setComputePipelineState(cptPSO)
            cptEncoder.setBuffer(pos_vel_bufIn, offset: 0, index: 0)
            cptEncoder.setBuffer(pos_vel_bufOut, offset: 0, index: 1)
            cptEncoder.setBuffer(dynamicSimCB, offset: simCBOffset, index: 2)
            cptEncoder.dispatchThreadgroups(numgroups, threadsPerThreadgroup: groupsize)
            cptEncoder.endEncoding()
            
            /// Delay getting the currentRenderPassDescriptor until we absolutely need it to avoid
            ///   holding onto the drawable and blocking the display pipeline any longer than necessary
            let renderPassDescriptor = view.currentRenderPassDescriptor
            
            if let renderPassDescriptor = renderPassDescriptor, let renderEncoder = commandBuffer.makeRenderCommandEncoder(descriptor: renderPassDescriptor) {
                
                /// Final pass rendering code here
                renderEncoder.label = "Primary Render Encoder"
                
                renderEncoder.pushDebugGroup("Draw Box")
                
                renderEncoder.setCullMode(.back)
                
                renderEncoder.setFrontFacing(.counterClockwise)
                
                renderEncoder.setRenderPipelineState(gfxPSO)
                
                renderEncoder.setDepthStencilState(depthState)
                
                renderEncoder.setVertexBuffer(dynamicUniformBuffer, offset:uniformBufferOffset, index: BufferIndex.uniforms.rawValue)
                renderEncoder.setFragmentBuffer(dynamicUniformBuffer, offset:uniformBufferOffset, index: BufferIndex.uniforms.rawValue)
                renderEncoder.setVertexBuffer(vertexBuf, offset: 0, index: 0)
               
                
                renderEncoder.setVertexBuffer(pos_vel_bufOut, offset: 0, index: 3)
                renderEncoder.setVertexBuffer(dynamicSimCB, offset: simCBOffset, index: 4)
                
                swap(&pos_vel_bufOut, &pos_vel_bufIn)
                
                renderEncoder.drawPrimitives(type: .triangleStrip, vertexStart: 0, vertexCount: 10, instanceCount: instanceCnt)
            
                
                renderEncoder.popDebugGroup()
                
                renderEncoder.endEncoding()
                
                if let drawable = view.currentDrawable {
                    commandBuffer.present(drawable)
                }
            }
            
            commandBuffer.commit()
            
            if resetBoids == true {
                commandBuffer.waitUntilCompleted()
                initPosVel()
                resetBoids = false
            }
        }
    }
    
    func mtkView(_ view: MTKView, drawableSizeWillChange size: CGSize) {
        /// Respond to drawable size or orientation changes here
        
        let aspect = Float(size.width) / Float(size.height)
        projectionMatrix = matrix_perspective_right_hand(fovyRadians: radians_from_degrees(65), aspectRatio:aspect, nearZ: 0.1, farZ: 200.0)
    }
}

// Generic matrix math utility functions
func matrix4x4_rotation(radians: Float, axis: float3) -> matrix_float4x4 {
    let unitAxis = normalize(axis)
    let ct = cosf(radians)
    let st = sinf(radians)
    let ci = 1 - ct
    let x = unitAxis.x, y = unitAxis.y, z = unitAxis.z
    return matrix_float4x4.init(columns:(vector_float4(    ct + x * x * ci, y * x * ci + z * st, z * x * ci - y * st, 0),
                                         vector_float4(x * y * ci - z * st,     ct + y * y * ci, z * y * ci + x * st, 0),
                                         vector_float4(x * z * ci + y * st, y * z * ci - x * st,     ct + z * z * ci, 0),
                                         vector_float4(                  0,                   0,                   0, 1)))
}

func matrix4x4_translation(_ translationX: Float, _ translationY: Float, _ translationZ: Float) -> matrix_float4x4 {
    return matrix_float4x4.init(columns:(vector_float4(1, 0, 0, 0),
                                         vector_float4(0, 1, 0, 0),
                                         vector_float4(0, 0, 1, 0),
                                         vector_float4(translationX, translationY, translationZ, 1)))
}

// Generic matrix math utility functions
func matrix_perspective_right_hand(fovyRadians fovy: Float, aspectRatio: Float, nearZ: Float, farZ: Float) -> matrix_float4x4 {
    let ys = 1 / tanf(fovy * 0.5)
    let xs = ys / aspectRatio
    let zs = farZ / (nearZ - farZ)
    return matrix_float4x4.init(columns:(vector_float4(xs,  0, 0,   0),
                                         vector_float4( 0, ys, 0,   0),
                                         vector_float4( 0,  0, zs, -1),
                                         vector_float4( 0,  0, zs * nearZ, 0)))
}

func radians_from_degrees(_ degrees: Float) -> Float {
    return (degrees / 180) * .pi
}
