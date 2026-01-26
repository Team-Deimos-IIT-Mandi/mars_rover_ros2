"use client"

import React, { useEffect, useState } from 'react';
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Camera, RefreshCw } from "lucide-react"
import { Button } from "@/components/ui/button"

interface CameraStreamProps {
    topic: string;
    name?: string;
    width?: number;
    height?: number;
}

export function CameraStream({ topic, name = "Camera Stream", width = 640, height = 480 }: CameraStreamProps) {
    const [streamUrl, setStreamUrl] = useState<string>("");
    const [timestamp, setTimestamp] = useState(Date.now());
    const [hasError, setHasError] = useState(false);

    useEffect(() => {
        // Construct the web_video_server URL
        // Assumes web_video_server is running on port 8080 relative to localhost client
        const hostname = window.location.hostname || 'localhost';
        // Using mjpeg type since ros_compressed requires image_transport plugins
        const url = `http://${hostname}:8080/stream?topic=${topic}&type=mjpeg&quality=80`;
        setStreamUrl(url);
        setHasError(false);
    }, [topic, timestamp]);

    const refreshStream = () => {
        setTimestamp(Date.now());
    }

    return (
        <Card className="w-full h-full bg-black/5 border-zinc-200/50 backdrop-blur-sm overflow-hidden shadow-sm hover:shadow-md transition-all duration-300">
            <CardHeader className="flex flex-row items-center justify-between space-y-0 pb-2">
                <CardTitle className="text-sm font-medium flex items-center gap-2">
                    <Camera className="h-4 w-4 text-primary" />
                    {name}
                </CardTitle>
                <Button variant="ghost" size="icon" className="h-4 w-4" onClick={refreshStream}>
                    <RefreshCw className="h-3 w-3" />
                    <span className="sr-only">Refresh</span>
                </Button>
            </CardHeader>
            <CardContent className="p-0 flex items-center justify-center bg-black aspect-video relative group">
                {streamUrl && !hasError ? (
                    <img
                        src={`${streamUrl}&t=${timestamp}`}
                        className="w-full h-full object-contain"
                        alt={`Stream for ${topic}`}
                        onError={(e) => {
                            setHasError(true);
                        }}
                    />
                ) : hasError ? (
                    <div className="text-muted-foreground text-center p-4 text-xs">
                        <Camera className="h-8 w-8 mx-auto mb-2 opacity-50" />
                        <div>No stream available</div>
                        <div className="text-[10px] mt-1 opacity-70">
                            Ensure simulation is running<br />
                            and publishing to {topic}
                        </div>
                    </div>
                ) : (
                    <div className="text-muted-foreground flex items-center gap-2">Loading Stream...</div>
                )}

                {/* Overlay info */}
                <div className="absolute top-2 right-2 opacity-0 group-hover:opacity-100 transition-opacity">
                    <Badge variant="secondary" className="bg-black/50 text-white hover:bg-black/70 font-mono text-xs">
                        {topic}
                    </Badge>
                </div>
            </CardContent>
        </Card>
    );
}
