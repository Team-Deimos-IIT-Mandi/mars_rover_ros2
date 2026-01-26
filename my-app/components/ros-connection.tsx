"use client"

import React, { createContext, useContext, useEffect, useState, ReactNode } from 'react';
import * as ROSLIB from 'roslib';

interface RosContextType {
    ros: ROSLIB.Ros | null;
    isConnected: boolean;
    error: string | null;
    url: string;
}

const RosContext = createContext<RosContextType>({
    ros: null,
    isConnected: false,
    error: null,
    url: 'ws://localhost:9090',
});

interface RosProviderProps {
    children: ReactNode;
    url?: string;
}

export function RosProvider({ children, url }: RosProviderProps) {
    // Dynamically determine ROS bridge URL based on current hostname
    const defaultUrl = typeof window !== 'undefined'
        ? `ws://${window.location.hostname}:9090`
        : 'ws://localhost:9090';

    const rosUrl = url || defaultUrl;

    const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
    const [isConnected, setIsConnected] = useState(false);
    const [error, setError] = useState<string | null>(null);

    useEffect(() => {
        console.log('🔌 Attempting to connect to ROS at:', rosUrl);

        // Handle ROSLIB import interop (esm/cjs mismatch)
        const RosConstructor = (ROSLIB as any).Ros || (ROSLIB as any).default?.Ros || ROSLIB;

        if (!RosConstructor) {
            setError('Failed to load ROSLIB');
            console.error('❌ Failed to load ROSLIB constructor');
            return;
        }

        const rosInstance = new RosConstructor({
            url: rosUrl,
        });

        rosInstance.on('connection', () => {
            console.log('✅ Connected to websocket server at', rosUrl);
            setIsConnected(true);
            setError(null);
        });

        rosInstance.on('error', (error: any) => {
            console.error('❌ Error connecting to websocket server:', error);
            console.error('   URL was:', rosUrl);
            setError(error.toString());
            setIsConnected(false);
        });

        rosInstance.on('close', () => {
            console.log('🔌 Connection to websocket server closed');
            setIsConnected(false);
        });

        setRos(rosInstance);

        return () => {
            rosInstance.close();
        };
    }, [rosUrl]);

    return (
        <RosContext.Provider value={{ ros, isConnected, error, url: rosUrl }}>
            {children}
        </RosContext.Provider>
    );
}

export function useRos() {
    return useContext(RosContext);
}
