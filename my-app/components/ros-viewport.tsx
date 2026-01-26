"use client"

import React, { useEffect, useRef, useState } from 'react';
import { useRos } from './ros-connection';
import * as ROSLIB from 'roslib';
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Box, Layers } from "lucide-react"

// ROS3D is often not TS friendly, might need 'require' or explicit any
// We installed 'ros3d' via npm?
let ROS3D: any;
try {
    ROS3D = require('ros3d');
} catch (e) {
    console.warn("ROS3D module not found, visualization will fail");
}

export function RosViewport() {
    const { ros, isConnected, url } = useRos();
    const viewerRef = useRef<HTMLDivElement>(null);
    const [status, setStatus] = useState("Initializing...");
    const viewerInstance = useRef<any>(null);

    useEffect(() => {
        if (!ros || !isConnected || !viewerRef.current || !ROS3D) return;
        if (viewerInstance.current) return; // Already initialized

        setStatus("Building 3D Scene...");

        try {
            // Create the main viewer.
            // width/height will be resized automatically usually if strict dims not set?
            // ROS3D.Viewer appends canvas to the div
            const viewer = new ROS3D.Viewer({
                divID: viewerRef.current.id,
                width: viewerRef.current.clientWidth,
                height: viewerRef.current.clientHeight,
                antialias: true,
                background: '#111111' // Dark background
            });

            // Add a grid.
            viewer.addObject(new ROS3D.Grid({
                color: '#333333',
                cellSize: 1.0,
                num_cells: 20
            }));

            // Setup a client to listen to TFs.
            // Requires tf2_web_republisher
            const tfClient = new ROSLIB.TFClient({
                ros: ros,
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0,
                fixedFrame: '/odom' // or /map 
            });

            // Setup the URDF client.
            const urdfClient = new ROS3D.UrdfClient({
                ros: ros,
                tfClient: tfClient,
                path: 'http://localhost:8080/', // web_video_server might not preserve file structure for meshes...
                // Typically meshes need to be served via HTTP. rosbridge doesn't serve files.
                // We might only get the TF shapes if meshes aren't served properly.
                // For now, assume TF shapes or primitives.
                rootObject: viewer.scene,
                loader: ROS3D.COLLADA_LOADER_2 // Fallback
            });

            // Marker Client for visualization markers (e.g. search Spiral)
            const markerClient = new ROS3D.MarkerClient({
                ros: ros,
                tfClient: tfClient,
                topic: '/visualization_marker',
                rootObject: viewer.scene
            });

            viewerInstance.current = viewer;
            setStatus("3D Scene Active");

            // Handle resize
            const resizeObserver = new ResizeObserver(() => {
                if (viewerRef.current) {
                    viewer.resize(viewerRef.current.clientWidth, viewerRef.current.clientHeight);
                }
            });
            resizeObserver.observe(viewerRef.current);

            return () => {
                resizeObserver.disconnect();
                // Cleanup logic if supported by ros3djs??
            }

        } catch (err) {
            console.error("Error init ROS3D", err);
            setStatus("3D Error");
        }

    }, [ros, isConnected]);

    return (
        <Card className="w-full h-full bg-zinc-950 border-zinc-800 flex flex-col">
            <CardHeader className="flex flex-row items-center justify-between py-2 px-4 bg-zinc-900 border-b border-zinc-800">
                <CardTitle className="text-sm font-medium flex items-center gap-2 text-zinc-100">
                    <Box className="h-4 w-4 text-purple-500" />
                    3D Simulation View
                </CardTitle>
                <div className="text-xs font-mono text-zinc-500">
                    {status}
                </div>
            </CardHeader>
            <CardContent className="p-0 flex-1 relative overflow-hidden">
                {!isConnected && (
                    <div className="absolute inset-0 flex items-center justify-center text-zinc-500 bg-black/50 z-10">
                        Waiting for ROS...
                    </div>
                )}
                <div id="ros3d-viewer" ref={viewerRef} className="w-full h-full" />
            </CardContent>
        </Card>
    );
}
