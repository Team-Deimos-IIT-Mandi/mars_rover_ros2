"use client"

import React, { useEffect, useRef, useState } from 'react';
import { useRos } from './ros-connection';
import * as ROSLIB from 'roslib';
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Map as MapIcon, Navigation } from "lucide-react"

interface MapViewProps {
    topic?: string;
    name?: string;
}

export function MapView({ topic = "/map", name = "Navigation Map" }: MapViewProps) {
    const { ros, isConnected } = useRos();
    const canvasRef = useRef<HTMLCanvasElement>(null);
    const [status, setStatus] = useState("Waiting for map...");

    useEffect(() => {
        if (!ros || !isConnected) return;

        const mapListener = new ROSLIB.Topic({
            ros: ros,
            name: topic,
            messageType: 'nav_msgs/OccupancyGrid',
            compression: 'png' // Optimization if server supports it, otherwise ignored usually
        });

        mapListener.subscribe((message: any) => {
            setStatus("Rendering map...");
            renderMap(message);
        });

        return () => {
            mapListener.unsubscribe();
        };
    }, [ros, isConnected, topic]);

    const renderMap = (message: any) => {
        const canvas = canvasRef.current;
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        if (!ctx) return;

        const width = message.info.width;
        const height = message.info.height;
        const data = message.data; // Int8Array usually, -1 unknown, 0 free, 100 occupied

        // Resize canvas if needed
        if (canvas.width !== width || canvas.height !== height) {
            canvas.width = width;
            canvas.height = height;
        }

        const imageData = ctx.createImageData(width, height);
        const buf = imageData.data;

        for (let i = 0; i < data.length; i++) {
            const val = data[i];
            const r = i * 4; // RGBA index

            if (val === -1) {
                // Unknown - Gray/Transparent
                buf[r] = 200;
                buf[r + 1] = 200;
                buf[r + 2] = 200;
                buf[r + 3] = 255;
            } else if (val === 0) {
                // Free - White
                buf[r] = 255;
                buf[r + 1] = 255;
                buf[r + 2] = 255;
                buf[r + 3] = 255;
            } else if (val === 100) {
                // Occupied - Black
                buf[r] = 0;
                buf[r + 1] = 0;
                buf[r + 2] = 0;
                buf[r + 3] = 255;
            } else {
                // Values in between - Grayscale
                const color = Math.floor(255 * (100 - val) / 100);
                buf[r] = color;
                buf[r + 1] = color;
                buf[r + 2] = color;
                buf[r + 3] = 255;
            }
        }

        // Flip Y axis because ROS maps originate bottom-left, Canvas is top-left
        // Wait, we can just render normal and use CSS scaleY(-1) or draw logic
        // Ideally we create a temp canvas to flip it
        ctx.putImageData(imageData, 0, 0);
        setStatus("Map Updated");
    };

    return (
        <Card className="w-full h-full bg-white border-zinc-200 shadow-sm">
            <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                <CardTitle className="text-sm font-medium flex items-center gap-2">
                    <MapIcon className="h-4 w-4 text-primary" />
                    {name}
                </CardTitle>
                <div className="text-xs text-muted-foreground">{status}</div>
            </CardHeader>
            <CardContent className="p-0 flex items-center justify-center bg-zinc-100 aspect-square overflow-hidden relative">
                {!isConnected && <div className="absolute inset-0 flex items-center justify-center text-muted-foreground bg-white/50 z-10">Disconnected</div>}
                {/* Transform to flip Y if needed, and scale to fit */}
                <div className="w-full h-full flex items-center justify-center p-4">
                    <canvas ref={canvasRef} className="max-w-full max-h-full object-contain" style={{ transform: 'scaleY(-1)' }} />
                </div>
            </CardContent>
        </Card>
    );
}
