"use client"

import { useState, useEffect, useRef, useCallback } from "react"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Badge } from "@/components/ui/badge"
import { Button } from "@/components/ui/button"
import { Alert, AlertDescription } from "@/components/ui/alert"
import { Activity, Info, Wifi, WifiOff, RefreshCw } from "lucide-react"

export function RosStatusPanel() {
  const [bridgeStatus, setBridgeStatus] = useState<"connected" | "disconnected" | "checking">("disconnected")
  const [lastChecked, setLastChecked] = useState<string>("")
  const isChecking = useRef(false)

  const checkBridgeStatus = useCallback(async () => {
    // Prevent multiple simultaneous checks
    if (isChecking.current) return
    isChecking.current = true
    setBridgeStatus("checking")

    try {
      const ws = new WebSocket("ws://localhost:9090")

      const timeout = setTimeout(() => {
        if (ws.readyState !== WebSocket.OPEN) {
          ws.close()
          setBridgeStatus("disconnected")
          setLastChecked(new Date().toLocaleTimeString())
          isChecking.current = false
        }
      }, 3000)

      ws.onopen = () => {
        clearTimeout(timeout)
        setBridgeStatus("connected")
        setLastChecked(new Date().toLocaleTimeString())
        ws.close()
        isChecking.current = false
      }

      ws.onerror = () => {
        clearTimeout(timeout)
        setBridgeStatus("disconnected")
        setLastChecked(new Date().toLocaleTimeString())
        isChecking.current = false
      }

      ws.onclose = () => {
        isChecking.current = false
      }

    } catch {
      setBridgeStatus("disconnected")
      setLastChecked(new Date().toLocaleTimeString())
      isChecking.current = false
    }
  }, [])

  useEffect(() => {
    // Check once on mount
    checkBridgeStatus()

    // Then check every 30 seconds
    const interval = setInterval(checkBridgeStatus, 30000)
    return () => clearInterval(interval)
  }, [checkBridgeStatus])

  return (
    <Card>
      <CardHeader>
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-2">
            <Activity className="h-5 w-5" />
            <CardTitle>ROS Bridge Status</CardTitle>
          </div>
          <Badge variant={bridgeStatus === "connected" ? "default" : bridgeStatus === "checking" ? "secondary" : "destructive"}>
            {bridgeStatus === "connected" ? (
              <><Wifi className="h-3 w-3 mr-1" /> Connected</>
            ) : bridgeStatus === "checking" ? (
              <><RefreshCw className="h-3 w-3 mr-1 animate-spin" /> Checking...</>
            ) : (
              <><WifiOff className="h-3 w-3 mr-1" /> Disconnected</>
            )}
          </Badge>
        </div>
        <CardDescription>WebSocket bridge for ROS2 (ws://localhost:9090)</CardDescription>
      </CardHeader>
      <CardContent className="space-y-4">
        {bridgeStatus === "disconnected" && (
          <Alert className="border-yellow-500/50 bg-yellow-500/10">
            <Info className="h-4 w-4" />
            <AlertDescription className="text-sm">
              <strong>ROS Bridge not running!</strong> Start it:
              <code className="block mt-1 bg-muted px-2 py-1 rounded text-xs">
                cd src && ./start-ros-bridge.sh
              </code>
            </AlertDescription>
          </Alert>
        )}

        {bridgeStatus === "connected" && (
          <Alert className="border-green-500/50 bg-green-500/10">
            <Wifi className="h-4 w-4" />
            <AlertDescription className="text-sm text-green-600">
              ROS Bridge is running!
            </AlertDescription>
          </Alert>
        )}

        <div className="flex items-center justify-between">
          <span className="text-xs text-muted-foreground">
            {lastChecked && `Checked: ${lastChecked}`}
          </span>
          <Button
            onClick={checkBridgeStatus}
            size="sm"
            variant="outline"
            disabled={bridgeStatus === "checking"}
          >
            <RefreshCw className={`h-3 w-3 mr-2 ${bridgeStatus === "checking" ? "animate-spin" : ""}`} />
            Check
          </Button>
        </div>
      </CardContent>
    </Card>
  )
}
