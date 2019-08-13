// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as child_process from "child_process";
import * as os from "os";
import * as vscode from "vscode";

import * as ros from "../ros";

export class ROS2 implements ros.ROSApi {
    private _context: vscode.ExtensionContext;
    private _env: any;

    public setContext(context: vscode.ExtensionContext, env: any) {
        this._context = context;
        this._env = env;
    }

    public getPackageNames(): Promise<string[]> {
        return new Promise((resolve, reject) => child_process.exec("ros2 pkg list", { env: this._env }, (err, out) => {
            if (!err) {
                const lines = out.trim().split(os.EOL).map(((line) => {
                    return line;
                }));
                resolve(lines);
            } else {
                reject(err);
            }
        }));
    }

    public getPackages(): Promise<{ [name: string]: string }> {
        return new Promise((resolve, reject) => {
            reject("not yet implemented");
        });
    }

    public getIncludeDirs(): Promise<string[]> {
        return new Promise((resolve, reject) => {
            reject("not yet implemented");
        });
    }

    public findPackageExecutables(packageName: string): Promise<string[]> {
        return new Promise((resolve, reject) => child_process.exec(`ros2 pkg executables ${packageName}`, { env: this._env }, (err, out) => {
            if (!err) {
                const lines = out.trim().split(os.EOL).map(((line) => {
                    const info: string[] = line.split(" ");
                    if (info.length === 2) {
                        // each line should contain exactly 2 strings separated by 1 space
                        return info;
                    }
                }));
    
                const packageInfoReducer = (acc: string[], cur: string[]) => {
                    const executableName: string = cur[1] as string;
                    acc.push(executableName);
                    return acc;
                };
                resolve(lines.reduce(packageInfoReducer, []));
            } else {
                reject(err);
            }
        }));
    }

    public findPackageLaunchFiles(packageName: string): Promise<string[]> {
        return new Promise((resolve, reject) => {
            reject("not yet implemented");
        });
    }

    public startCore() {
        // not yet implemented.
        return;
    }

    public stopCore() {
        // not yet implemented.
        return;
    }

    public activateCoreMonitor(): vscode.Disposable {
        // not yet implemented.
        return null;
    }

    public showCoreMonitor() {
        // not yet implemented.
        return;
    }

    public activateRosrun(packageName: string, executableName:string, argument: string): vscode.Terminal {
        let terminal = vscode.window.createTerminal({
            env: this._env,
            name: "ros2 run",
        });
        terminal.sendText(`ros2 run ${packageName} ${executableName} ${argument}`);
        return terminal;
    }

    public activateRoslaunch(launchFilepath: string, argument: string): vscode.Terminal {
        // not yet implemented.
        return null;
    }
}
