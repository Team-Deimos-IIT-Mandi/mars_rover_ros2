"use client"

import React, { useEffect, useRef } from 'react';
import { useRos } from './ros-connection';
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Box } from "lucide-react"

export function Ros3DViewer() {
    const { ros, isConnected } = useRos();
    const viewerRef = useRef<HTMLDivElement>(null);
    const ros3dViewerRef = useRef<any>(null);

    useEffect(() => {
        if (!viewerRef.current || !ros || !isConnected) return;

        // Dynamically import ROS3D to avoid SSR issues
        import('ros3d').then((ROS3D) => {
            if (!viewerRef.current || ros3dViewerRef.current) return;

            // Create the 3D viewer
            const viewer = new ROS3D.Viewer({
                divID: viewerRef.current.id,
                width: viewerRef.current.clientWidth,
                height: viewerRef.current.clientHeight,
                antialias: true,
                background: '#111111'
            });

            // Add grid
            viewer.addObject(new ROS3D.Grid({
                num_cells: 20,
                color: '#444444'
            }));

            // Setup TF client
            const tfClient = new ROS3D.TFClient({
                ros: ros,
                fixedFrame: 'map',
                angularThres: 0.01,
                transThres: 0.01,
                rate: 10.0
            });

            // Add robot model (URDF)
            const urdfClient = new ROS3D.UrdfClient({
                ros: ros,
                tfClient: tfClient,
                path: 'http://localhost:8080/',
                rootObject: viewer.scene,
                loader: ROS3D.COLLADA_LOADER_2
            });

            // Add laser scan visualization
            const laserScan = new ROS3D.LaserScan({
                ros: ros,
                tfClient: tfClient,
                topic: '/scan',
                rootObject: viewer.scene,
                material: { size: 0.05, color: 0xff0000 }
            });

            // Add path visualization (global plan)
            const globalPath = new ROS3D.Path({
                ros: ros,
                tfClient: tfClient,
                topic: '/plan',
                rootObject: viewer.scene,
                color: 0x00ff00
            });

            // Add local path
            const localPath = new ROS3D.Path({
                ros: ros,
                tfClient: tfClient,
                topic: '/local_plan',
                rootObject: viewer.scene,
                color: 0xffff00
            });

            ros3dViewerRef.current = viewer;

            // Handle window resize
            const handleResize = () => {
                if (viewerRef.current && ros3dViewerRef.current) {
                    ros3dViewerRef.current.resize(
                        viewerRef.current.clientWidth,
                        viewerRef.current.clientHeight
                    );
                }
            };
            window.addEventListener('resize', handleResize);

            return () => {
                window.removeEventListener('resize', handleResize);
            };
        }).catch(err => {
            console.error('Failed to load ROS3D:', err);
        });

        return () => {
            if (ros3dViewerRef.current) {
                ros3dViewerRef.current = null;
            }
        };
    }, [ros, isConnected]);

    return (
        <Card className="w-full h-full bg-zinc-900 border-zinc-800 flex flex-col">
            <CardHeader className="pb-2 flex-shrink-0">
                <CardTitle className="text-sm font-medium flex items-center gap-2 text-zinc-100">
                    <Box className="h-4 w-4 text-primary" />
                    3D Navigation View
                    {!isConnected && (
                        <span className="text-xs text-red-500 ml-2">(Disconnected)</span>
                    )}
                </CardTitle>
            </CardHeader>
            <CardContent className="p-0 flex-1">
                <div
                    id="ros3d-viewer"
                    ref={viewerRef}
                    className="w-full h-full bg-zinc-950"
                    style={{ minHeight: '500px' }}
                />
            </CardContent>
        </Card>
    );
}
