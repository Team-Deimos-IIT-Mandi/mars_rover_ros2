"use client"

import React, { useEffect, useState } from 'react';
import { useRos } from './ros-connection';
import * as ROSLIB from 'roslib';
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { MapPin } from "lucide-react"
import dynamic from 'next/dynamic';

// Dynamic import for the Map client component to avoid SSR issues
const GpsMapClient = dynamic(() => import('./gps-map-client'), {
    ssr: false,
    loading: () => <div className="w-full h-full flex items-center justify-center bg-zinc-100 text-zinc-400">Loading Map...</div>
});

export function GpsMap() {
    const { ros, isConnected } = useRos();
    const [position, setPosition] = useState<[number, number]>([49.90027, 8.90036]); // Default per launch file
    const [path, setPath] = useState<[number, number][]>([]);
    const [fixStatus, setFixStatus] = useState("Waiting for GPS...");

    useEffect(() => {
        if (!ros || !isConnected) return;

        const gpsListener = new ROSLIB.Topic({
            ros: ros,
            name: '/gps/fix',
            messageType: 'sensor_msgs/NavSatFix'
        });

        gpsListener.subscribe((message: any) => {
            const lat = message.latitude;
            const lon = message.longitude;

            if (lat && lon) {
                const newPos: [number, number] = [lat, lon];
                setPosition(newPos);

                // Simple path filtering (only add if moved significantly)
                setPath(prev => {
                    const last = prev[prev.length - 1];
                    if (!last || (Math.abs(last[0] - lat) > 0.00001 || Math.abs(last[1] - lon) > 0.00001)) {
                        return [...prev, newPos].slice(-500); // Keep last 500 points
                    }
                    return prev;
                });

                const status = message.status?.status;
                setFixStatus(status >= 0 ? "GPS Locked" : "No Fix");
            }
        });

        return () => {
            gpsListener.unsubscribe();
        };
    }, [ros, isConnected]);

    return (
        <Card className="w-full h-full bg-white border-zinc-200 shadow-sm overflow-hidden flex flex-col">
            <CardHeader className="flex flex-row items-center justify-between py-2 px-4 bg-zinc-50 border-b">
                <CardTitle className="text-sm font-medium flex items-center gap-2">
                    <MapPin className="h-4 w-4 text-blue-500" />
                    Global Positioning
                </CardTitle>
                <div className="text-xs font-mono text-muted-foreground">
                    {position[0].toFixed(5)}, {position[1].toFixed(5)} ({fixStatus})
                </div>
            </CardHeader>
            <CardContent className="p-0 flex-1 relative z-0">
                <GpsMapClient center={position} zoom={18} path={path} />
            </CardContent>
        </Card>
    );
}
