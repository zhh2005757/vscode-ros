// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

import * as path from "path";
import * as vscode from "vscode";
import * as pfs from "../promise-fs";
import * as extension from "../extension";
import * as telemetry from "../telemetry-helper";
import * as catkin_make from "./catkin-make";
import * as catkin_tools from "./catkin-tools";

export abstract class BuildTool {
    static current: BuildTool;
    static registerTaskProvider(): vscode.Disposable {
        return this.current._registerTaskProvider();
    }

    static async createPackage(context: vscode.ExtensionContext) {
        const reporter = telemetry.getReporter(context);
        reporter.sendTelemetryCommand(extension.Commands.CreateCatkinPackage);
        return this.current._createPackage();
    }

    protected abstract _registerTaskProvider(): vscode.Disposable;
    protected abstract async _createPackage(): Promise<void>;
}

class NotImplementedBuildTool extends BuildTool {
    protected _registerTaskProvider(): vscode.Disposable {
        return null;
    }

    protected async _createPackage(): Promise<void> {
        return;
    }
}

class CatkinCmakeBuildTool extends BuildTool {
    protected _registerTaskProvider(): vscode.Disposable {
        return vscode.workspace.registerTaskProvider("catkin", new catkin_make.CatkinMakeProvider());
    }

    protected async _createPackage(): Promise<void> {
        return catkin_make.createPackage();
    }
}

class CatkinToolsBuildTool extends BuildTool {
    protected _registerTaskProvider(): vscode.Disposable {
        return vscode.workspace.registerTaskProvider("catkin", new catkin_tools.CatkinToolsProvider());
    }

    protected async _createPackage(): Promise<void> {
        return catkin_tools.createPackage();
    }
}

BuildTool.current = new NotImplementedBuildTool();

/**
 * Determines build system and workspace path in use by checking for unique
 * auto-generated files.
 */
export async function determineBuildTool(dir: string): Promise<boolean> {
    while (dir && path.dirname(dir) !== dir) {
        if (await pfs.exists(`${dir}/.catkin_workspace`)) {
            extension.setBaseDir(dir);
            BuildTool.current = new CatkinCmakeBuildTool();
            return true;
        } else if (await pfs.exists(`${dir}/.catkin_tools`)) {
            extension.setBaseDir(dir);
            BuildTool.current = new CatkinToolsBuildTool();
            return true;
        }

        dir = path.dirname(dir);
    }
    return false;
}
