"use client"

import React, { useState } from 'react';
import { Dialog, DialogContent, DialogDescription, DialogHeader, DialogTitle, DialogFooter } from "@/components/ui/dialog"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { RadioGroup, RadioGroupItem } from "@/components/ui/radio-group"
import { MapPin, Rocket } from "lucide-react"
import dynamic from 'next/dynamic';

const GpsMapClient = dynamic(() => import('./gps-map-client'), {
    ssr: false,
    loading: () => <div className="h-[300px] w-full bg-muted animate-pulse rounded-md" />
});

interface ArucoLaunchDialogProps {
    open: boolean;
    onOpenChange: (open: boolean) => void;
    onLaunch: (args: { lat: string; lon: string }) => void;
}

export function ArucoLaunchDialog({ open, onOpenChange, onLaunch }: ArucoLaunchDialogProps) {
    const [mode, setMode] = useState<"default" | "manual" | "map">("default");
    const [lat, setLat] = useState("49.90027");
    const [lon, setLon] = useState("8.90036");

    const handleLaunch = () => {
        let args = {};

        if (mode === "default") {
            args = { lat: "49.90027", lon: "8.90036" };
        } else {
            args = { lat, lon };
        }

        onLaunch(args);
        onOpenChange(false);
    };

    return (
        <Dialog open={open} onOpenChange={onOpenChange}>
            <DialogContent className="max-w-4xl max-h-[90vh] overflow-y-auto">
                <DialogHeader>
                    <DialogTitle className="flex items-center gap-2">
                        <Rocket className="h-5 w-5 text-primary" />
                        ArUco Mission Launch Configuration
                    </DialogTitle>
                    <DialogDescription>
                        Configure GPS target coordinates for the autonomous ArUco search mission
                    </DialogDescription>
                </DialogHeader>

                <div className="grid grid-cols-1 lg:grid-cols-2 gap-6 py-4">
                    {/* Left: Options */}
                    <div className="space-y-4">
                        <div>
                            <Label className="text-base font-semibold mb-3 block">Target Selection Mode</Label>
                            <RadioGroup value={mode} onValueChange={(v: "default" | "manual" | "map") => setMode(v)} className="space-y-3">
                                <div className="flex items-center space-x-2">
                                    <RadioGroupItem value="default" id="d1" />
                                    <Label htmlFor="d1" className="font-normal cursor-pointer">Default Target (30m North)</Label>
                                </div>
                                <div className="flex items-center space-x-2">
                                    <RadioGroupItem value="manual" id="d2" />
                                    <Label htmlFor="d2" className="font-normal cursor-pointer">Enter Coordinates Manually</Label>
                                </div>
                                <div className="flex items-center space-x-2">
                                    <RadioGroupItem value="map" id="d3" />
                                    <Label htmlFor="d3" className="font-normal cursor-pointer">Click on Map</Label>
                                </div>
                            </RadioGroup>
                        </div>

                        <div className={`space-y-3 transition-opacity ${mode === 'default' ? 'opacity-50 pointer-events-none' : 'opacity-100'}`}>
                            <div className="space-y-2">
                                <Label htmlFor="lat">Target Latitude</Label>
                                <Input
                                    id="lat"
                                    value={lat}
                                    onChange={(e) => setLat(e.target.value)}
                                    className="font-mono"
                                    disabled={mode === 'default'}
                                />
                            </div>
                            <div className="space-y-2">
                                <Label htmlFor="lon">Target Longitude</Label>
                                <Input
                                    id="lon"
                                    value={lon}
                                    onChange={(e) => setLon(e.target.value)}
                                    className="font-mono"
                                    disabled={mode === 'default'}
                                />
                            </div>
                        </div>

                        <div className="p-3 bg-muted rounded-lg text-sm">
                            <div className="font-medium mb-1">Selected Target:</div>
                            <div className="font-mono text-xs">
                                {mode === 'default' ? '49.90027, 8.90036 (Default)' : `${lat}, ${lon}`}
                            </div>
                        </div>
                    </div>

                    {/* Right: Map */}
                    <div className="space-y-2">
                        <Label className="text-base font-semibold">Interactive Map</Label>
                        <div className="border rounded-lg overflow-hidden h-[350px]">
                            <GpsMapClient
                                center={[parseFloat(lat) || 49.9, parseFloat(lon) || 8.9]}
                                zoom={18}
                                path={[]}
                                onClick={(newLat, newLon) => {
                                    if (mode === 'map') {
                                        setLat(newLat.toFixed(6));
                                        setLon(newLon.toFixed(6));
                                    }
                                }}
                            />
                        </div>
                        <p className="text-xs text-muted-foreground">
                            {mode === 'map' ? '✓ Click anywhere on the map to set target' : 'Select "Click on Map" mode to interact'}
                        </p>
                    </div>
                </div>

                <DialogFooter>
                    <Button variant="outline" onClick={() => onOpenChange(false)}>
                        Cancel
                    </Button>
                    <Button onClick={handleLaunch} className="bg-primary">
                        <Rocket className="h-4 w-4 mr-2" />
                        Launch Mission
                    </Button>
                </DialogFooter>
            </DialogContent>
        </Dialog>
    );
}
