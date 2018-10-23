//
//  Shaders.metal
//  BoidSimulation Shared
//
//  Created by Peng Liu on 5/13/18.
//  Copyright © 2018 PLStudio. All rights reserved.
//

// File for Metal kernel and shader functions

#include <metal_stdlib>
#include <simd/simd.h>

// Including header shared between this Metal shader code and Swift/C code executing Metal API commands
#import "ShaderTypes.h"

using namespace metal;

typedef struct
{
    float3 position [[attribute(VertexAttributePosition)]];
} Vertex;

typedef struct
{
    float4 position [[position]];
    float3 col;
} ColorInOut;

float3 palette(float t, float3 a, float3 b, float3 c, float3 d){
    return a + b * cos(6.28318 * (c * t + d));
}

vertex ColorInOut vsRender(Vertex in [[stage_in]],
                               constant Uniforms & uniforms [[ buffer(BufferIndexUniforms) ]],
                               const device FishData* pos_vel [[buffer(3)]],
                               constant SimCB &simCB [[buffer(4)]],
                               ushort iid [[ instance_id ]])
{
    ColorInOut out;
    
    float3 pos = pos_vel[iid].pos;
    float3 vel = pos_vel[iid].vel;
    
    float velLen = length(vel);
    vel /= velLen;
    
    float3 fup = float3(0,1,0);
    if (abs(dot(fup, vel)) > 0.95) {
        fup = normalize(float3(pos.x, 0, pos.z));
    }
    // Right vector for each fish
    float3 right = cross(fup, vel);
    float3 up = cross(right, vel);
    // Rotation matrix to pose each fish heading forward
    float3x3 rotMatrix = {vel, up, right};
    rotMatrix = transpose(rotMatrix);
    // Set the fish size
    float size = simCB.fFishSize;
    // Calculate color index for later velocity color mapping
    float vidx = (velLen - simCB.fMinSpeed) / (simCB.fMaxSpeed - simCB.fMinSpeed);
    
    float3 outPos = in.position * float3(1.5, 1, 1) * size * rotMatrix;
    
    float4 position = float4(outPos + pos, 1.0);
    out.position = uniforms.projectionMatrix * uniforms.modelViewMatrix * position;
    
    // Calculate the color based on speed
    out.col = palette(vidx, float3(0.5), float3(0.5), float3(1.0), float3(0.0, 0.33, 0.67));

    return out;
}

fragment float4 fsRender(ColorInOut in [[stage_in]],
                               constant Uniforms & uniforms [[ buffer(BufferIndexUniforms) ]])
{

    float4 colorSample = float4(in.col.xyz, 0.0);

    return float4(colorSample);
}

//------------------------------------------------------------------------------
// Utility Functions for Compute Shader
//------------------------------------------------------------------------------
// Shader static constant
const constant float softeningSquared = 0.00125*0.00125;
const constant float softening = 0.00125;
//const constant float bondaryFactor = 10.0f;

// Calculate the force from border
float3 BoundaryCorrection(float3 localPos, float3 localVel)
{
    float3 probePos = localPos + localVel * 0.5;
    return probePos;
}

// Calculate the force to avoid other fish
float3 Avoidance(float fAvoidanceFactor, float3 localPos, float3 velDir, float3 avoidPos, float distSqr)
{
    float3 OP = avoidPos - localPos;
    float t = dot(OP, velDir);
    float3 tPos = localPos + velDir * t;
    float3 force = tPos - avoidPos;
    float forceLenSqr = dot(force, force) + softeningSquared;
    return fAvoidanceFactor * force / (forceLenSqr * distSqr);
}

// Calculate separation force
float3 Seperation(float fSeperationFactor, float3 neighborDir, float3 neighborVel, float invDist)
{
    float3 neighborVelSqr = dot(neighborVel, neighborVel) + softeningSquared;
    float3 invNeighborVelLen = 1.0f / sqrt(neighborVelSqr);
    float3 neighborVelDir = neighborVel * invNeighborVelLen;
    float directionFactor = abs(dot(neighborDir, neighborVelDir)) + softening;
    return -fSeperationFactor *
    neighborDir * invDist * invDist * (1 + 3 * directionFactor);
}

// Calculate cohesion force
float3 Cohesion(float fCohesionFactor, float3 localPos, float3 avgPos)
{
    float3 delta = avgPos - localPos;
    float deltaSqr = dot(delta, delta) + softeningSquared;
    float invDelta = 1.0f / sqrt(deltaSqr);
    return fCohesionFactor * delta * invDelta;
}

// Calculate alignment force
float3 Alignment(float fAlignmentFactor, float3 localVel, float3 avgVel)
{
    float3 delta = avgVel - localVel;
    float deltaSqr = dot(delta, delta) + softeningSquared;
    float invDelta = 1.0f / sqrt(deltaSqr);
    return fAlignmentFactor * delta * invDelta;
}

// Calculate seeking force
float3 Seekings(float fSeekingFactor, float fMaxSpeed, float3 localPos, float3 localVel, float3 seekPos)
{
    float3 delta = seekPos - localPos;
    float deltaSqr = dot(delta, delta) + softeningSquared;
    float invDelta = 1.0f / sqrt(deltaSqr);
    //float3 desiredVel = delta * invDelta * fMaxSpeed - localVel;
    return fSeekingFactor * delta * invDelta;
}

float3 Seeking(float fSeekingFactor, float fMaxSpeed, float3 vLocalPos, float3 vLocalVel, float3 vSeekPos)
{
    float3 vDelta = normalize(vSeekPos - vLocalPos);
    float3 vDesired = vDelta * fMaxSpeed;
    return fSeekingFactor * (vDesired - vLocalVel);
}

// Calculate flee force
float3 Flee(float fFleeFactor, float fMaxSpeed, float3 localPos, float3 localVel, float3 fleePos)
{
    float3 delta = localPos - fleePos;
    float deltaSqr = dot(delta, delta) + softeningSquared;
    float invDelta = 1.0f / sqrt(deltaSqr);
    float3 desiredVel = delta * fMaxSpeed;
    return fFleeFactor*(desiredVel - localVel) *invDelta*invDelta;
}

// Calculate border force from planes
float3 PlaneVelCorrection(float3 f3xyzExpand, float3 probePos)
{
    float3 diff = max(float3(0, 0, 0), probePos - f3xyzExpand);
    probePos = probePos - diff;
    return probePos;
}

// Calculate border force from edges
float2 EdgeVelCorrection(float2 probeToEdge, float cornerRadius, float2 probePos)
{
    if (all(probeToEdge < float2(0, 0))) {
        float dist = length(probeToEdge);
        if (dist > cornerRadius) {
            float2 moveDir = normalize(probeToEdge);
            probePos += moveDir * (dist - cornerRadius);
        }
    }
    return probePos;
}

// Calculate border force from corners
float3 CornerVelCorrection(float3 probeToCorner, float cornerRadius, float3 probPos)
{
    if (all(probeToCorner < float3(0, 0, 0))) {
        float dist = length(probeToCorner);
        if (dist > cornerRadius) {
            float3 moveDir = normalize(probeToCorner);
            probPos += moveDir * (dist - cornerRadius);
        }
    }
    return probPos;
}

float3 BorderVelCorrection(float3 f3xyzExpand, float fDeltaT, float3 pos, float3 vel)
{
    float speed = length(vel);
    float probeDist = speed * 25 * fDeltaT;
    float3 probePos = pos + 25 * fDeltaT * vel;
    float cornerRadius = probeDist * 1.5f;
    int3 posState = int3(pos > float3(0, 0, 0));
    int3 convert = posState * 2 - int3(1, 1, 1);
    float3 mirrorProbePos = probePos * float3(convert);
    float3 cornerSphereCenterPos = f3xyzExpand - cornerRadius;
    float3 probeToCorner = cornerSphereCenterPos - mirrorProbePos;
    // For corners
    CornerVelCorrection(probeToCorner, cornerRadius, mirrorProbePos);
    // For edges
    mirrorProbePos.xy = EdgeVelCorrection(probeToCorner.xy, cornerRadius, mirrorProbePos.xy);
    mirrorProbePos.xz = EdgeVelCorrection(probeToCorner.xz, cornerRadius, mirrorProbePos.xz);
    mirrorProbePos.yz = EdgeVelCorrection(probeToCorner.yz, cornerRadius, mirrorProbePos.yz);
    // For planes
    mirrorProbePos = PlaneVelCorrection(f3xyzExpand, mirrorProbePos);
    // Get true new probe pos
    probePos = mirrorProbePos * float3(convert);
    // Get new vel
    vel = speed * normalize(probePos - pos);
    return vel;
}


kernel void csSimulationI(const device FishData* pos_velIn [[buffer(0)]],
                         device FishData* pos_velOut [[buffer(1)]],
                         constant SimCB &simCB [[buffer(2)]],
                         const uint i [[thread_position_in_grid]])
{
    if (i >= simCB.uInstanceCnt) return;
    pos_velOut[i].pos = pos_velIn[i].pos + 0.1*sin(simCB.fDeltaT + i/20.f);
    pos_velOut[i].vel = float3(1);
}

kernel void csSimulation(const device FishData* oldVP [[buffer(0)]],
                          device FishData* newVP [[buffer(1)]],
                          constant SimCB &simCB [[buffer(2)]],
                          const uint tGlobalID [[thread_position_in_grid]],
                          const uint tGroupID [[thread_position_in_threadgroup]],
                          const uint gID [[threadgroup_position_in_grid]])
{
    threadgroup FishData sharedFishVP[BLOCK_SIZE];
    
    // Each thread of the CS updates one fish
    FishData localVP = oldVP[tGlobalID];
    // Transform to local space
    localVP.pos -= simCB.f3CenterPos;
    
    float predatorFactor = 1;
    float vDT = simCB.fVisionDist;
    float vAT = simCB.fVisionAngleCos;
    
    // Local variable to store temporary values
    // Keep track of all forces for this fish
    float3 accForce = 0;
    // Accumulate neighbor fish pos for neighbor ave pos calculation
    float3 accPos = 0;
    // Accumulate neighbor fish vel for neighbor ave vel calculation
    float3 accVel = 0;
    // Number of near by fish (neighbor) for ave data calculation
    uint accCount = 0;
    
    float scalarVel = sqrt(dot(localVP.vel, localVP.vel));
    float3 velDir = localVP.vel / scalarVel;
    
    for (uint tile = 0; tile < uint(simCB.uInstanceCnt / BLOCK_SIZE); tile++) {
        // Cache a tile of particles to shared memory to increase IO efficiency
        // uint currentTileStart = tile * BLOCK_SIZE;
        
        sharedFishVP[tGroupID] = oldVP[tile * BLOCK_SIZE + tGroupID];
        //sharedFishVP[tGroupID] = oldVP[0];
        // Transform to local space
        sharedFishVP[tGroupID].pos -= simCB.f3CenterPos;

        threadgroup_barrier(mem_flags::mem_threadgroup);

        //[unroll]
        for (uint counter = 0; counter < BLOCK_SIZE; counter++) {
            // Calculate distance
            float3 vPos = sharedFishVP[counter].pos - localVP.pos;
            float distSqr = dot(vPos, vPos) + softeningSquared;
            float dist = sqrt(distSqr);
            float invDist = 1.0f / dist;
            // Calculate angle between vel and dist dir
            float3 neighborDir = vPos * invDist;

            float cosAngle = dot(velDir, neighborDir);
            // Doing one to one interaction based on visibility
            if (dist <= vDT && cosAngle >= vAT) {
                // accumulate neighbor fish pos
                accPos += sharedFishVP[counter].pos;
                // accumulate neighbor fish vel
                accVel += sharedFishVP[counter].vel;
                // counting neighbors for calculating avg PV
                accCount += 1;
                // Add separation and avoidance force
                accForce += Seperation(simCB.fSeperationFactor, neighborDir, sharedFishVP[counter].vel, invDist) * predatorFactor;
                accForce += Avoidance(simCB.fAvoidanceFactor, localVP.pos, velDir, sharedFishVP[counter].pos, distSqr) * predatorFactor;
            }
        }
    }
    
    threadgroup_barrier(mem_flags::mem_threadgroup);

    // Calculate average pos and vel of neighbor fish
    float3 avgPos = float3(0, 0, 0);
    if (accCount != 0) {
        avgPos = accPos / accCount;
        float3 avgVel = accVel / accCount;
        // Convert flee source pos to local space
        float3 localFleeSourcePos = simCB.f3FleeSourcePos - simCB.f3CenterPos;
        // Add cohesion alignment forces
        accForce += Cohesion(simCB.fCohesionFactor, localVP.pos, avgPos + normalize(localVP.vel) * 0.2f);
        accForce += Alignment(simCB.fAlignmentFactor, localVP.vel, avgVel) * predatorFactor;
        accForce += Flee(simCB.fFleeFactor, simCB.fMaxSpeed, localVP.pos, localVP.vel, localFleeSourcePos) * predatorFactor;
        accForce += (avgPos - localVP.pos) * 0.5f;
    }

    // Convert seek source pos to local space
    float3 seekPos = simCB.f3SeekSourcePos - simCB.f3CenterPos;
    float maxForce = simCB.fMaxForce;
    float maxSpeed = simCB.fMaxSpeed;
    float minSpeed = simCB.fMinSpeed;
    float3 seek = Seeking(simCB.fSeekingFactor, simCB.fMaxSpeed, localVP.pos, localVP.vel, seekPos);
    accForce += ((accCount == 0) ? 100.f * seek : seek);
    float accForceSqr = dot(accForce, accForce) + softeningSquared;
    float invForceLen = 1.0f / sqrt(accForceSqr);
    float3 forceDir = accForce * invForceLen;
    if (accForceSqr > maxForce * maxForce) {
        accForce = forceDir * simCB.fMaxForce;
    }

    localVP.vel += accForce * simCB.fDeltaT;
    float velAfterSqr = dot(localVP.vel, localVP.vel);
    float invVelLen = 1.0f / sqrt(velAfterSqr);
    if (velAfterSqr > maxSpeed * maxSpeed) {
        localVP.vel = localVP.vel * invVelLen * maxSpeed;
    } else if (velAfterSqr < minSpeed * minSpeed) {
        localVP.vel = localVP.vel * invVelLen * minSpeed;
    }
    localVP.vel = BorderVelCorrection(simCB.f3xyzExpand, simCB.fDeltaT, localVP.pos, localVP.vel);
    localVP.pos.xyz += localVP.vel.xyz * simCB.fDeltaT;

    // Convert the result pos back to world space
    newVP[tGlobalID].pos = localVP.pos + simCB.f3CenterPos;
    newVP[tGlobalID].vel = localVP.vel;
}
