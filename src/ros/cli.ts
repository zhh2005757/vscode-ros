// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";

import * as extension from "../extension";
import * as telemetry from "../telemetry-helper";
import { rosApi } from "./ros"

export async function rosrun(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter(context);
    reporter.sendTelemetryCommand(extension.Commands.Rosrun);

    const terminal = await preparerosrun();
    terminal.show();
}

async function preparerosrun(): Promise<vscode.Terminal> {
    const getPackages = rosApi.getPackages();
    const packageName = await vscode.window.showQuickPick(getPackages.then((packages: { [name: string]: string }) => {
        return Object.keys(packages);
    }), {
        placeHolder: "Choose a package",
    });
    if (packageName !== undefined) {
        let basenames = (files: string[]) => files.map((file) => path.basename(file));

        const executables = rosApi.findPackageExecutables(packageName).then(basenames);
        let target = await vscode.window.showQuickPick(executables, { placeHolder: "Choose an executable" });
        let argument = await vscode.window.showInputBox({ placeHolder: "Enter any extra arguments" });
        return rosApi.activateRosrun(packageName, target, argument);
    } else {
        // none of the packages selected, error!
    }
}

export async function roslaunch(context: vscode.ExtensionContext) {
    const reporter = telemetry.getReporter(context);
    reporter.sendTelemetryCommand(extension.Commands.Roslaunch);

    let terminal = await prepareroslaunch();
    terminal.show();
}

async function prepareroslaunch(): Promise<vscode.Terminal> {
    const getPackages = rosApi.getPackages();
    const packageName = await vscode.window.showQuickPick(getPackages.then((packages: { [name: string]: string }) => {
        return Object.keys(packages);
    }), {
        placeHolder: "Choose a package",
    });
    if (packageName !== undefined) {
        const launchFiles = await rosApi.findPackageLaunchFiles(packageName);
        const launchFileBasenames = launchFiles.map((filename) => path.basename(filename));
        let target = await vscode.window.showQuickPick(launchFileBasenames, { placeHolder: "Choose a launch file" });
        let argument = await vscode.window.showInputBox({ placeHolder: "Enter any extra arguments" });
        return rosApi.activateRoslaunch(launchFiles[launchFileBasenames.indexOf(target)], argument);
    } else {
        // none of the packages selected, error!
    }
}
