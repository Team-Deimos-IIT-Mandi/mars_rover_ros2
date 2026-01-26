"use client"

import React from 'react';
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Monitor } from "lucide-react"

export function RvizVncViewer() {
    // noVNC URL with auto-connect and fullscreen scaling
    const vncUrl = `http://localhost:6080/vnc.html?autoconnect=true&resize=scale&reconnect=true`;

    return (
        <div className="w-full h-full bg-black">
            <iframe
                src={vncUrl}
                className="w-full h-full border-0"
                title="RViz VNC Viewer"
                allow="clipboard-read; clipboard-write"
                style={{ display: 'block', margin: 0, padding: 0 }}
            />
        </div>
    );
}
