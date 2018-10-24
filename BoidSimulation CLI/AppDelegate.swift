//
//  AppDelegate.swift
//  BoidSimulation CLI
//
//  Created by Peng Liu on 10/23/18.
//  Copyright © 2018 PLStudio. All rights reserved.
//

import Cocoa
import AppKit

class AppDelegate: NSObject, NSApplicationDelegate {
    
    lazy var window = NSWindow(contentRect: NSMakeRect(100, 100, 200, 200),
                               styleMask: .titled, backing: .buffered, defer: false, screen: nil)
    var gameViewCtrl: GameViewController!
    
    override init() {
        super.init()
        
        let viewCtrl = GameViewController()
        
        window.contentViewController = viewCtrl
        gameViewCtrl = viewCtrl
        
    }
    func applicationDidFinishLaunching(_ notification: Notification) {
        let button = NSButton(frame: NSMakeRect(20, 20, 100, 32))
        button.title = "Reset Boids"
        button.target = self
        button.action = #selector(onClick)
        window.contentView?.addSubview(button)
        window.makeKeyAndOrderFront(nil)
    }
    
    @IBAction func onClick(_: Any) {
        NSSound.beep()
        gameViewCtrl.resetBoids()
        NSLog("Boids Reset!")
    }
    
}
