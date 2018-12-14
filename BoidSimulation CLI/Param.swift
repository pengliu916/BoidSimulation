//
//  Param.swift
//  BoidSimulation CLI
//
//  Created by Peng Liu on 10/25/18.
//  Copyright © 2018 PLStudio. All rights reserved.
//
import Cocoa

enum Param {
    case Help
    case ResetButton
    case Width (reso: UInt32)
    case Height (reso: UInt32)
    
    var identifier: String {
        switch self {
        case .Help: return "-h"
        case .ResetButton: return "-reset"
        case .Width: return "-width"
        case .Height: return "-height"
        }
    }
    
    static func parsing(args:[String]) -> [Param]? {
        var params = [Param]()
        var argsItr = args.makeIterator()
        // skip the first param, its the executable name...
        _ = argsItr.next()
        while let arg = argsItr.next() {
            switch arg {
            case Param.Help.identifier:
                return [Param.Help]
            case Param.ResetButton.identifier:
                params.append(Param.ResetButton)
            case Param.Width(reso:0).identifier:
                guard let _arg = argsItr.next(), let width = UInt32(_arg) else {return nil}
                params.append(Param.Width(reso: width))
            case Param.Height(reso: 0).identifier:
                guard let _arg = argsItr.next(), let height = UInt32(_arg) else {return nil}
                params.append(Param.Height(reso: height))
            default:
                return nil
            }
        }
        return params
    }
    
    static func printHelp() {
        let exeName = (CommandLine.arguments[0] as NSString).lastPathComponent
        let msg = """
        usage:
        \(exeName) -width 800 -height 600
        or
        \(exeName) -width 800
        or
        \(exeName) -height 600
        or
        \(exeName) -h to show usage information
        type \(exeName) without an option to enter default mode.
        """
        print(msg)
    }
}
