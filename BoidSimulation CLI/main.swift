//
//  main.swift
//  BoidSimulation CLI
//
//  Created by Peng Liu on 10/23/18.
//  Copyright © 2018 PLStudio. All rights reserved.
//

import Cocoa

var width: UInt32 = 800
var height: UInt32 = 600
var reset = false

let app = NSApplication.shared
if let params = Param.parsing(args: CommandLine.arguments) {
    for param in params {
        switch param {
        case let Param.Width(w):
            width = w
        case Param.Height(let h):
            height = h
        case Param.ResetButton:
            reset = true
        case Param.Help:
            Param.printHelp()
            exit(0)
        }
    }
} else {
    Param.printHelp()
    exit(0)
}

app.delegate = AppDelegate(width, height, reset)
app.run()

