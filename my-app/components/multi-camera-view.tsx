"use client"

import React from 'react';
import { CameraStream } from './camera-stream';
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Eye } from "lucide-react"

export function MultiCameraView() {
    return (
        <Card className="w-full h-full bg-zinc-900 border-zinc-800 flex flex-col">
            <CardHeader className="pb-2 flex-shrink-0">
                <CardTitle className="text-sm font-medium flex items-center gap-2 text-zinc-100">
                    <Eye className="h-4 w-4 text-primary" />
                    Sensor Array
                </CardTitle>
            </CardHeader>
            <CardContent className="p-2 flex flex-col gap-2 flex-1 overflow-auto">
                {/* Main Camera (Front) - Responsive height */}
                <div className="relative w-full h-[250px] sm:h-[350px] md:h-[500px] border border-zinc-700 rounded overflow-auto">
                    <CameraStream topic="/camera_1/image_raw" name="Front Camera" />
                </div>

                {/* Side Cameras - Stack on mobile, grid on larger screens */}
                <div className="grid grid-cols-1 sm:grid-cols-2 gap-2 min-h-[200px] sm:h-[300px] md:h-[400px]">
                    <div className="relative overflow-auto border border-zinc-700 rounded min-h-[150px]">
                        <CameraStream topic="/camera_2/image_raw" name="Left Cam" />
                    </div>
                    <div className="relative overflow-auto border border-zinc-700 rounded min-h-[150px]">
                        <CameraStream topic="/rgbd_camera/image" name="Right Depth Cam" />
                    </div>
                </div>
            </CardContent>
        </Card>
    );
}
