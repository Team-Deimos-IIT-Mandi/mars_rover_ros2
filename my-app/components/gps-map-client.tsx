"use client"

import React, { useEffect } from 'react';
import { MapContainer, TileLayer, Marker, Popup, Polyline, useMap } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';

// Fix for default marker icons in Leaflet with Webpack/Next.js
// @ts-ignore
delete L.Icon.Default.prototype._getIconUrl;
L.Icon.Default.mergeOptions({
    iconRetinaUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon-2x.png',
    iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png',
    shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png',
});

function MapUpdater({ center }: { center: [number, number] }) {
    const map = useMap();
    useEffect(() => {
        map.setView(center);
    }, [center, map]);
    return null;
}

interface GpsMapClientProps {
    center: [number, number];
    zoom: number;
    path: [number, number][];
    onClick?: (lat: number, lon: number) => void;
}

function MapEvents({ onClick }: { onClick?: (lat: number, lon: number) => void }) {
    const map = useMap();
    useEffect(() => {
        if (!onClick) return;

        const handleClick = (e: L.LeafletMouseEvent) => {
            onClick(e.latlng.lat, e.latlng.lng);
        };

        map.on('click', handleClick);
        return () => {
            map.off('click', handleClick);
        };
    }, [map, onClick]);
    return null;
}

export default function GpsMapClient({ center, zoom, path, onClick }: GpsMapClientProps) {
    return (
        <MapContainer center={center} zoom={zoom} style={{ height: '100%', width: '100%' }}>
            <TileLayer
                attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
                url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
            />
            <Marker position={center}>
                <Popup>
                    Current Rover Position
                </Popup>
            </Marker>
            <Polyline positions={path} color="blue" />
            <MapUpdater center={center} />
            <MapEvents onClick={onClick} />
        </MapContainer>
    );
}
