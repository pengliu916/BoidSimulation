//
//  GameViewController.swift
//  BoidSimulation CLI
//
//  Created by Peng Liu on 10/23/18.
//  Copyright © 2018 PLStudio. All rights reserved.
//

import Cocoa
import MetalKit

// Our macOS specific view controller
class GameViewController: NSViewController {
    
    var renderer: Renderer!
    var mtkView: MTKView!
    
    override func loadView() {
        self.view = MTKView(frame: NSMakeRect(100, 100, 200, 200))
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        guard let mtkView = self.view as? MTKView else {
            print("View attached to GameViewController is not an MTKView")
            return
        }
        
        // Select the device to render with.  We choose the default device
        guard let defaultDevice = MTLCreateSystemDefaultDevice() else {
            print("Metal is not supported on this device")
            return
        }
        
        mtkView.device = defaultDevice
        
        guard let newRenderer = Renderer(metalKitView: mtkView) else {
            print("Renderer cannot be initialized")
            return
        }
        
        renderer = newRenderer
        
        renderer.mtkView(mtkView, drawableSizeWillChange: mtkView.drawableSize)
        
        mtkView.delegate = renderer
    }
    
    override func viewWillAppear() {
        self.view.window?.styleMask.insert(NSWindow.StyleMask.fullSizeContentView)
        self.view.window?.toolbar?.isVisible = false
        self.view.window?.titleVisibility = .hidden
        self.view.window?.titlebarAppearsTransparent = true
        self.view.window?.isMovableByWindowBackground = true
        self.view.window?.standardWindowButton(NSWindow.ButtonType.closeButton)?.isHidden = true
        self.view.window?.standardWindowButton(NSWindow.ButtonType.miniaturizeButton)?.isHidden = true
        self.view.window?.standardWindowButton(NSWindow.ButtonType.zoomButton)?.isHidden = true
    }
    
    func resetBoids() {
        renderer.resetBoids = true
    }
}
