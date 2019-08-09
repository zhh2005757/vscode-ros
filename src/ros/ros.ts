// Copyright (c) Andrew Short. All rights reserved.
// Licensed under the MIT License.

import * as vscode from "vscode";

import * as ros1 from './ros1'

export interface ROSApi {
    /**
     * Set additional context.
     */
    setContext: (context: vscode.ExtensionContext, env: any) => void;

    /**
     * Gets a map of package names to paths.
     */
    getPackages: () => Promise<{ [name: string]: string }>;

    /**
     * Gets a list of CMake include paths.
     */
    getIncludeDirs: () => Promise<string[]>;

    /**
     * list full paths to all executables inside a package
     * @param packageName 
     */
    findPackageExecutables: (packageName: string) => Promise<string[]>;

    /**
     * list all .launch files inside a package
     * @param packageName 
     */
    findPackageLaunchFiles: (packageName: string) => Promise<string[]>;

    /**
     * Start ROS Core
     */
    startCore: () => void;

    /**
     * Stop ROS Core
     */
    stopCore: () => void;

    /**
     * Activate ROS Core monitor.
     */
    activateCoreMonitor: () => vscode.Disposable;

    /**
     * Bring up ROS Core monitor GUI.
     */
    showCoreMonitor: () => void;

    /**
     * Activate a terminal for rosrun.
     */
    activateRosrun: (packageName: string, executableName:string, argument: string) => vscode.Terminal;

    /**
     * Activate a terminal for roslaunch.
     */
    activateRoslaunch: (launchFilepath: string, argument: string) => vscode.Terminal;
}

export let rosApi: ROSApi = new ros1.ROS1();
