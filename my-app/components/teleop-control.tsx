"use client"

import React, { useState } from 'react';
import { useRos } from './ros-connection';
import * as ROSLIB from 'roslib';
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { ArrowUp, ArrowDown, ArrowLeft, ArrowRight, StopCircle, Gamepad2 } from "lucide-react"

interface TeleopControlProps {
    topic?: string;
}

export function TeleopControl({ topic = "/cmd_vel" }: TeleopControlProps) {
    const { ros, isConnected } = useRos();
    const [linearSpeed, setLinearSpeed] = useState(0.5);
    const [angularSpeed, setAngularSpeed] = useState(1.0);

    const timerRef = React.useRef<NodeJS.Timeout | null>(null);

    const startMoving = (linear: number, angular: number) => {
        if (!ros || !isConnected) return;

        // Clear any existing timer
        if (timerRef.current) clearInterval(timerRef.current);

        const cmdVel = new ROSLIB.Topic({
            ros: ros,
            name: topic,
            messageType: 'geometry_msgs/Twist'
        });

        const twist = {
            linear: { x: linear, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: angular }
        };

        // Publish immediately
        cmdVel.publish(twist);

        // Publish periodically to keep robot alive (watchdog)
        timerRef.current = setInterval(() => {
            cmdVel.publish(twist);
        }, 100);
    };

    const stopMoving = () => {
        if (timerRef.current) {
            clearInterval(timerRef.current);
            timerRef.current = null;
        }

        if (!ros || !isConnected) return;

        const cmdVel = new ROSLIB.Topic({
            ros: ros,
            name: topic,
            messageType: 'geometry_msgs/Twist'
        });

        // Send zero velocity
        const twist = {
            linear: { x: 0, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 }
        };
        cmdVel.publish(twist);
    };

    return (
        <Card className="w-full h-full bg-white border-zinc-200 shadow-sm">
            <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                <CardTitle className="text-sm font-medium flex items-center gap-2">
                    <Gamepad2 className="h-4 w-4 text-primary" />
                    Teleoperation
                </CardTitle>
                <div className={`h-2 w-2 rounded-full ${isConnected ? 'bg-green-500' : 'bg-red-500'}`} />
            </CardHeader>
            <CardContent className="h-full flex flex-col items-center justify-center gap-4 pb-6">
                {/* Forward */}
                <Button
                    variant="outline"
                    size="icon"
                    className="h-12 w-12 rounded-full active:bg-primary active:text-white transition-colors"
                    onMouseDown={() => startMoving(linearSpeed, 0)}
                    onMouseUp={stopMoving}
                    onMouseLeave={stopMoving}
                    onTouchStart={(e) => { e.preventDefault(); startMoving(linearSpeed, 0); }}
                    onTouchEnd={(e) => { e.preventDefault(); stopMoving(); }}
                >
                    <ArrowUp className="h-6 w-6" />
                </Button>

                <div className="flex gap-4">
                    {/* Left */}
                    <Button
                        variant="outline"
                        size="icon"
                        className="h-12 w-12 rounded-full active:bg-primary active:text-white transition-colors"
                        onMouseDown={() => startMoving(0, angularSpeed)}
                        onMouseUp={stopMoving}
                        onMouseLeave={stopMoving}
                        onTouchStart={(e) => { e.preventDefault(); startMoving(0, angularSpeed); }}
                        onTouchEnd={(e) => { e.preventDefault(); stopMoving(); }}
                    >
                        <ArrowLeft className="h-6 w-6" />
                    </Button>

                    {/* Stop (explicit) */}
                    <Button
                        variant="destructive"
                        size="icon"
                        className="h-12 w-12 rounded-full shadow-md"
                        onClick={stopMoving}
                    >
                        <StopCircle className="h-6 w-6" />
                    </Button>

                    {/* Right */}
                    <Button
                        variant="outline"
                        size="icon"
                        className="h-12 w-12 rounded-full active:bg-primary active:text-white transition-colors"
                        onMouseDown={() => startMoving(0, -angularSpeed)}
                        onMouseUp={stopMoving}
                        onMouseLeave={stopMoving}
                        onTouchStart={(e) => { e.preventDefault(); startMoving(0, -angularSpeed); }}
                        onTouchEnd={(e) => { e.preventDefault(); stopMoving(); }}
                    >
                        <ArrowRight className="h-6 w-6" />
                    </Button>
                </div>

                {/* Backward */}
                <Button
                    variant="outline"
                    size="icon"
                    className="h-12 w-12 rounded-full active:bg-primary active:text-white transition-colors"
                    onMouseDown={() => startMoving(-linearSpeed, 0)}
                    onMouseUp={stopMoving}
                    onMouseLeave={stopMoving}
                    onTouchStart={(e) => { e.preventDefault(); startMoving(-linearSpeed, 0); }}
                    onTouchEnd={(e) => { e.preventDefault(); stopMoving(); }}
                >
                    <ArrowDown className="h-6 w-6" />
                </Button>
            </CardContent>
        </Card>
    );
}
