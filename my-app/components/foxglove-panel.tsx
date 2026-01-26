"use client"

import React from 'react';
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Navigation } from "lucide-react"

export function FoxglovePanel() {
    // Foxglove Studio URL with WebSocket connection to foxglove_bridge
    const foxgloveUrl = `https://studio.foxglove.dev/?ds=foxglove-websocket&ds.url=ws://${typeof window !== 'undefined' ? window.location.hostname : 'localhost'}:8765`;

    return (
        <Card className="w-full h-full bg-zinc-900 border-zinc-800 flex flex-col">
            <CardHeader className="pb-2 flex-shrink-0">
                <CardTitle className="text-sm font-medium flex items-center gap-2 text-zinc-100">
                    <Navigation className="h-4 w-4 text-primary" />
                    Navigation Visualization
                </CardTitle>
            </CardHeader>
            <CardContent className="p-6 flex-1 flex flex-col items-center justify-center gap-4">
                <div className="text-center space-y-4">
                    <div className="text-zinc-400">
                        Foxglove Studio cannot be embedded due to security restrictions.
                    </div>
                    <a
                        href={foxgloveUrl}
                        target="_blank"
                        rel="noopener noreferrer"
                        className="inline-flex items-center gap-2 px-6 py-3 bg-primary text-primary-foreground rounded-lg hover:bg-primary/90 transition-colors font-medium"
                    >
                        <Navigation className="h-5 w-5" />
                        Open Foxglove Studio in New Tab
                    </a>
                    <div className="text-xs text-zinc-500 max-w-md">
                        This will open Foxglove Studio and automatically connect to your rover at ws://localhost:8765.
                        You'll see paths, costmaps, waypoints, laser scans, and all navigation data.
                    </div>
                    <div className="text-xs text-yellow-500 mt-4">
                        Note: Make sure foxglove_bridge is running (port 8765)
                    </div>
                </div>
            </CardContent>
        </Card>
    );
}
