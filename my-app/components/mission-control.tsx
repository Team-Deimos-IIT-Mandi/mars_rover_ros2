"use client"

import React, { useState } from 'react';
import { useRos } from './ros-connection';
import { useProcessManager } from "@/hooks/use-process-manager";
import * as ROSLIB from 'roslib';
import { Card, CardContent, CardHeader, CardTitle, CardDescription } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { MapPin, Navigation, Crosshair, Target, Rocket } from "lucide-react"
import { RadioGroup, RadioGroupItem } from "@/components/ui/radio-group"
import { Badge } from "@/components/ui/badge"

// Dynamic map import
import dynamic from 'next/dynamic';
const GpsMapClient = dynamic(() => import('./gps-map-client'), {
    ssr: false,
    loading: () => <div className="h-[400px] w-full bg-muted animate-pulse rounded-md" />
});

export function MissionControl() {
    const { ros, isConnected } = useRos();
    const [lat, setLat] = useState("49.90027");
    const [lon, setLon] = useState("8.90036");
    const [status, setStatus] = useState("Ready");

    const publishGoal = (targetLat: string, targetLon: string) => {
        if (!ros || !isConnected) {
            setStatus("ROS Disconnected");
            return;
        }

        // Typically mission goals are sent as NavSatFix to a specific topic
        // OR if using the ArUco mission structure, maybe we restart the mission?
        // User said "coordinates will be published"
        // Let's publish to /mission/goal (NavSatFix) and /goal_pose (PoseStamped in odom) as fallback
        // Assuming mission_initializer or a script listens to something.
        // But strictly, we can publish to /goal_pose if we convert it, but we can't easily convert GPS to Odom on frontend without TF.
        // So we publish GPS.

        const gpsGoal = new ROSLIB.Topic({
            ros: ros,
            name: '/mission_goal_gps',
            messageType: 'sensor_msgs/NavSatFix'
        });

        const msg = {
            latitude: parseFloat(targetLat),
            longitude: parseFloat(targetLon),
            altitude: 0.0,
            position_covariance: [0, 0, 0, 0, 0, 0, 0, 0, 0],
            position_covariance_type: 0,
            status: { status: 0, service: 1 },
            header: { frame_id: 'gps_link' }
        };

        gpsGoal.publish(msg);
        setStatus(`Goal Sent: ${targetLat}, ${targetLon}`);
    };

    return (
        <div className="grid grid-cols-1 lg:grid-cols-3 gap-6 h-full">
            {/* Controls */}
            <Card className="lg:col-span-1">
                <CardHeader>
                    <CardTitle className="flex items-center gap-2">
                        <Navigation className="h-5 w-5 text-primary" />
                        Mission Parameters
                    </CardTitle>
                    <CardDescription>Set GPS target for autonomous ArUco search</CardDescription>
                </CardHeader>
                <CardContent className="space-y-4">
                    <div className="space-y-2">
                        <Label>Target Latitude</Label>
                        <Input value={lat} onChange={(e) => setLat(e.target.value)} />
                    </div>
                    <div className="space-y-2">
                        <Label>Target Longitude</Label>
                        <Input value={lon} onChange={(e) => setLon(e.target.value)} />
                    </div>

                    <div className="flex gap-2 pt-2">
                        <Button className="flex-1" onClick={() => publishGoal(lat, lon)}>
                            <Crosshair className="h-4 w-4 mr-2" />
                            Set Goal
                        </Button>
                        <Button variant="outline" onClick={() => { setLat("49.90027"); setLon("8.90036"); }}>
                            Default
                        </Button>
                    </div>

                    <div className="p-2 bg-muted rounded text-xs font-mono">
                        Status: {status}
                    </div>
                </CardContent>
            </Card>

            {/* Interactive Map */}
            <Card className="lg:col-span-2 min-h-[500px]">
                <CardHeader>
                    <CardTitle>Tactical Map</CardTitle>
                </CardHeader>
                <CardContent className="h-[500px] p-0 relative">
                    <GpsMapClient
                        center={[parseFloat(lat) || 49.9, parseFloat(lon) || 8.9]}
                        zoom={18}
                        path={[]}
                    // We need to extend GpsMapClient to accept an onClick handler
                    // For now, it's just viewing. I'll need to modify GpsMapClient to support 'onClick' props
                    />
                    <div className="absolute bottom-4 right-4 bg-white/90 p-2 rounded text-xs text-black z-[1000]">
                        Click map to set goal (Coming Soon)
                    </div>
                </CardContent>
            </Card>
        </div>
    );
}
