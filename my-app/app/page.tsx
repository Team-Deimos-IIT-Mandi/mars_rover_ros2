"use client"

import { useState, useEffect } from "react"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Alert, AlertDescription } from "@/components/ui/alert"
import { Separator } from "@/components/ui/separator"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import { Play, Square, AlertTriangle, Activity, Zap, FileText, Eye, Cog, Map, MapPin, Navigation, Camera, Gamepad2, MonitorPlay } from "lucide-react"
import { useProcessManager } from "@/hooks/use-process-manager"
import { RosStatusPanel } from "@/components/ros-status-panel"
import { LogViewer } from "@/components/log-viewer"

// Mars Rover Launch Files - ROS2 Humble
const ROVER_LAUNCH_FILES = {
  hardware: {
    name: "Hardware Control",
    description: "CAN motor controllers + USB sensors (Jetson Nano deployment)",
    file: "hardware.launch.py",
    icon: Cog,
    category: "hardware",
    critical: true,
  },
  gazebo_sim: {
    name: "Gazebo Simulation",
    description: "Full Gazebo simulation environment for testing",
    file: "gazebo_sim.launch.py",
    icon: Eye,
    category: "simulation",
  },
  complete_system: {
    name: "Complete System",
    description: "Full rover system with navigation, control, and visualization",
    file: "complete_system.launch.py",
    icon: Navigation,
    category: "core",
    critical: true,
  },
  gps_navigation: {
    name: "GPS Navigation",
    description: "GPS waypoint navigation with Nav2 stack",
    file: "gps_navigation.launch.py",
    icon: MapPin,
    category: "navigation",
  },
  dual_ekf_navsat: {
    name: "State Estimation",
    description: "Dual EKF for odometry and GPS sensor fusion",
    file: "dual_ekf_navsat.launch.py",
    icon: Activity,
    category: "navigation",
  },
  aruco: {
    name: "ArUco Detection",
    description: "ArUco marker detection and pose estimation",
    file: "aruco.launch.py",
    icon: Camera,
    category: "vision",
  },
  mapping: {
    name: "SLAM Mapping",
    description: "Simultaneous localization and mapping with SLAM Toolbox",
    file: "mapping.launch.py",
    icon: Map,
    category: "navigation",
  },
  display: {
    name: "RViz Visualization",
    description: "3D visualization of robot model, sensors, and environment",
    file: "display.launch.py",
    icon: Eye,
    category: "visualization",
  },
  controllers: {
    name: "Motor Controllers",
    description: "Differential drive controllers (hardware or simulation)",
    file: "controllers.launch.py",
    icon: Cog,
    category: "control",
  },
  gz_plus_control: {
    name: "Gazebo + Control",
    description: "Gazebo simulation with ros2_control integration",
    file: "gz_plus_control.launch.py",
    icon: Gamepad2,
    category: "simulation",
  },
  foxglove_viz: {
    name: "Foxglove Visualization",
    description: "Foxglove Studio WebSocket bridge for advanced visualization",
    file: "foxglove_viz.launch.py",
    icon: MonitorPlay,
    category: "visualization",
  },
  complete_foxglove: {
    name: "Complete + Foxglove",
    description: "Full system with Foxglove bridge for web-based monitoring",
    file: "complete_foxglove.launch.py",
    icon: Zap,
    category: "core",
  },
  foxglove_sim: {
    name: "Foxglove Simulation",
    description: "Simulation environment with Foxglove visualization",
    file: "foxglove_sim.launch.py",
    icon: Eye,
    category: "simulation",
  },
}

export default function MarsRoverDashboard() {
  const { processes, loading, error, startProcess, stopProcess, startAll, stopAll, emergencyStop, refresh } =
    useProcessManager()

  const [systemStatus, setSystemStatus] = useState<"offline" | "partial" | "online">("offline")
  const [emergencyActive, setEmergencyActive] = useState(false)

  // Calculate system status based on process states
  useEffect(() => {
    const processStates = Object.values(processes)
    const runningCount = processStates.filter((p) => p.status === "running").length
    const totalCount = processStates.length

    if (runningCount === 0) {
      setSystemStatus("offline")
    } else if (runningCount === totalCount) {
      setSystemStatus("online")
    } else {
      setSystemStatus("partial")
    }
  }, [processes])

  const handleEmergencyStop = async () => {
    setEmergencyActive(true)
    await emergencyStop()
    setTimeout(() => setEmergencyActive(false), 3000)
  }

  const getStatusColor = (status: string) => {
    switch (status) {
      case "running":
        return "bg-green-500"
      case "starting":
        return "bg-yellow-500"
      case "error":
        return "bg-red-500"
      default:
        return "bg-gray-500"
    }
  }

  const getSystemStatusColor = () => {
    switch (systemStatus) {
      case "online":
        return "text-green-400"
      case "partial":
        return "text-yellow-400"
      default:
        return "text-red-400"
    }
  }

  const getCategoryProcesses = (category: string) => {
    return Object.entries(ROVER_LAUNCH_FILES).filter(([key, process]) => process.category === category)
  }

  if (loading) {
    return (
      <div className="min-h-screen bg-background text-foreground flex items-center justify-center">
        <div className="flex items-center gap-2">
          <FileText className="h-4 w-4 animate-spin" />
          <span>Loading Mars Rover systems...</span>
        </div>
      </div>
    )
  }

  return (
    <div className="min-h-screen bg-background text-foreground p-6">
      <div className="max-w-7xl mx-auto space-y-6">
        {/* Header */}
        <div className="flex items-center justify-between">
          <div>
            <h1 className="text-3xl font-bold text-balance">Mars Rover Control Dashboard</h1>
            <p className="text-muted-foreground mt-1">ROS2 Humble - Process Management & Real-time Control</p>
            <p className="text-xs text-muted-foreground mt-1">Workspace: {process.env.ROS_WORKSPACE || '/home/anish/test/mars_rover_ros2'}</p>
          </div>
          <div className="flex items-center gap-4">
            <Button variant="outline" size="sm" onClick={refresh} className="flex items-center gap-2 bg-transparent">
              <FileText className="h-4 w-4" />
              Refresh
            </Button>
            <div className="flex items-center gap-2">
              <div className={`w-3 h-3 rounded-full ${getSystemStatusColor().replace("text-", "bg-")}`} />
              <span className={`font-medium ${getSystemStatusColor()}`}>
                System {systemStatus.charAt(0).toUpperCase() + systemStatus.slice(1)}
              </span>
            </div>
          </div>
        </div>

        {/* Error Alert */}
        {error && (
          <Alert className="border-destructive bg-destructive/10">
            <AlertTriangle className="h-4 w-4" />
            <AlertDescription className="text-destructive-foreground">{error}</AlertDescription>
          </Alert>
        )}

        {/* Emergency Stop Alert */}
        {emergencyActive && (
          <Alert className="border-destructive bg-destructive/10">
            <AlertTriangle className="h-4 w-4" />
            <AlertDescription className="text-destructive-foreground">
              Emergency stop activated! All processes are being terminated.
            </AlertDescription>
          </Alert>
        )}

        {/* Main Content Tabs */}
        <Tabs defaultValue="overview" className="space-y-6">
          <TabsList className="grid w-full grid-cols-2">
            <TabsTrigger value="overview" className="flex items-center gap-2">
              <Activity className="h-4 w-4" />
              Launch Files
            </TabsTrigger>
            <TabsTrigger value="logs" className="flex items-center gap-2">
              <FileText className="h-4 w-4" />
              Logs
            </TabsTrigger>
          </TabsList>

          <TabsContent value="overview" className="space-y-6">
            {/* ROS Status Panel */}
            <RosStatusPanel />

            {/* Master Controls */}
            <Card>
              <CardHeader>
                <CardTitle className="flex items-center gap-2">
                  <Zap className="h-5 w-5" />
                  Master Controls
                </CardTitle>
                <CardDescription>Control all rover launch files simultaneously</CardDescription>
              </CardHeader>
              <CardContent>
                <div className="flex gap-4">
                  <Button onClick={startAll} className="flex items-center gap-2" disabled={emergencyActive}>
                    <Play className="h-4 w-4" />
                    Start All Systems
                  </Button>
                  <Button variant="secondary" onClick={stopAll} className="flex items-center gap-2">
                    <Square className="h-4 w-4" />
                    Stop All Systems
                  </Button>
                  <Separator orientation="vertical" className="h-8" />
                  <Button variant="destructive" onClick={handleEmergencyStop} className="flex items-center gap-2">
                    <AlertTriangle className="h-4 w-4" />
                    Emergency Stop
                  </Button>
                </div>
              </CardContent>
            </Card>

            {/* Launch Files by Category */}
            {["core", "hardware", "simulation", "control", "navigation", "vision", "visualization"].map((category) => {
              const categoryProcesses = getCategoryProcesses(category)
              if (categoryProcesses.length === 0) return null

              return (
                <Card key={category}>
                  <CardHeader>
                    <CardTitle className="capitalize">{category} Launch Files</CardTitle>
                    <CardDescription>{categoryProcesses.length} launch file(s) in this category</CardDescription>
                  </CardHeader>
                  <CardContent>
                    <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                      {categoryProcesses.map(([key, launch]) => {
                        const state = processes[key]
                        const Icon = launch.icon

                        return (
                          <div key={key} className="flex flex-col p-4 border rounded-lg space-y-3">
                            <div className="flex items-start justify-between">
                              <div className="flex items-center gap-3">
                                <div className="p-2 rounded-lg bg-primary/10">
                                  <Icon className="h-5 w-5 text-primary" />
                                </div>
                                <div>
                                  <div className="font-medium text-sm">{launch.name}</div>
                                  {launch.critical && (
                                    <Badge variant="destructive" className="text-xs mt-1">Critical</Badge>
                                  )}
                                </div>
                              </div>
                              <div className={`w-2 h-2 rounded-full ${getStatusColor(state?.status || "stopped")}`} />
                            </div>
                            <div className="text-xs text-muted-foreground">{launch.description}</div>
                            <div className="text-xs font-mono text-muted-foreground bg-muted/50 px-2 py-1 rounded">
                              {launch.file}
                            </div>
                            <div className="flex items-center gap-2 pt-2">
                              <Badge
                                variant={
                                  state?.status === "running" ? "default" :
                                    state?.status === "stopping" ? "destructive" :
                                      "secondary"
                                }
                                className="text-xs flex-1 justify-center"
                              >
                                {state?.status === "stopping" ? "Stopping..." : (state?.status || "stopped")}
                              </Badge>
                              {state?.status === "running" ? (
                                <Button
                                  variant="outline"
                                  size="sm"
                                  onClick={() => stopProcess(key)}
                                  className="flex-1"
                                >
                                  <Square className="h-3 w-3 mr-1" />
                                  Stop
                                </Button>
                              ) : state?.status === "stopping" ? (
                                <Button
                                  variant="ghost"
                                  size="sm"
                                  disabled={true}
                                  className="flex-1"
                                >
                                  <Square className="h-3 w-3 mr-1 animate-pulse" />
                                  Stopping (10s)...
                                </Button>
                              ) : (
                                <Button
                                  size="sm"
                                  onClick={() => startProcess(key)}
                                  disabled={state?.status === "starting" || state?.status === "stopping" || emergencyActive}
                                  className="flex-1"
                                >
                                  <Play className="h-3 w-3 mr-1" />
                                  Launch
                                </Button>
                              )}
                            </div>
                          </div>
                        )
                      })}
                    </div>
                  </CardContent>
                </Card>
              )
            })}
          </TabsContent>



          <TabsContent value="logs">
            <LogViewer />
          </TabsContent>
        </Tabs>

        {/* System Information */}
        <Card>
          <CardHeader>
            <CardTitle>System Information</CardTitle>
            <CardDescription>Current ROS2 environment and connection status</CardDescription>
          </CardHeader>
          <CardContent>
            <div className="grid grid-cols-1 md:grid-cols-4 gap-4 text-sm">
              <div>
                <span className="font-medium">ROS Distribution:</span>
                <div className="text-muted-foreground">Humble</div>
              </div>
              <div>
                <span className="font-medium">Workspace:</span>
                <div className="text-muted-foreground text-xs">{process.env.ROS_WORKSPACE || '/home/anish/test/mars_rover_ros2'}</div>
              </div>
              <div>
                <span className="font-medium">Active Processes:</span>
                <div className="text-muted-foreground">
                  {Object.values(processes).filter((p) => p.status === "running").length} /{" "}
                  {Object.keys(ROVER_LAUNCH_FILES).length}
                </div>
              </div>
              <div>
                <span className="font-medium">Rover Model:</span>
                <div className="text-muted-foreground">mars_rover</div>
              </div>
            </div>
          </CardContent>
        </Card>
      </div>
    </div>
  )
}