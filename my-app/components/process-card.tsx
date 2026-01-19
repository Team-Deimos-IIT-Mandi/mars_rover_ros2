"use client"

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { Play, Square, type LucideIcon } from "lucide-react"

interface ProcessCardProps {
  name: string
  description: string
  command: string
  icon: LucideIcon
  status: "stopped" | "starting" | "running" | "error"
  pid?: number
  uptime?: number
  onStart: () => void
  onStop: () => void
  disabled?: boolean
}

export function ProcessCard({
  name,
  description,
  command,
  icon: Icon,
  status,
  pid,
  uptime,
  onStart,
  onStop,
  disabled = false,
}: ProcessCardProps) {
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

  return (
    <Card className="relative">
      <CardHeader className="pb-3">
        <div className="flex items-center justify-between">
          <div className="flex items-center gap-3">
            <div className="p-2 rounded-lg bg-primary/10">
              <Icon className="h-5 w-5 text-primary" />
            </div>
            <div>
              <CardTitle className="text-lg">{name}</CardTitle>
              <CardDescription className="text-sm">{description}</CardDescription>
            </div>
          </div>
          <div className="flex items-center gap-2">
            <div className={`w-2 h-2 rounded-full ${getStatusColor(status)}`} />
            <Badge variant={status === "running" ? "default" : "secondary"}>{status}</Badge>
          </div>
        </div>
      </CardHeader>
      <CardContent className="space-y-4">
        <div className="text-sm text-muted-foreground font-mono bg-muted/20 p-2 rounded">{command}</div>

        {pid && (
          <div className="flex gap-4 text-sm">
            <span>
              PID: <code className="font-mono">{pid}</code>
            </span>
            <span>
              Uptime: <code className="font-mono">{uptime || 0}s</code>
            </span>
          </div>
        )}

        <div className="flex gap-2">
          {status === "running" ? (
            <Button variant="secondary" size="sm" onClick={onStop} className="flex items-center gap-2">
              <Square className="h-3 w-3" />
              Stop
            </Button>
          ) : (
            <Button
              size="sm"
              onClick={onStart}
              disabled={status === "starting" || disabled}
              className="flex items-center gap-2"
            >
              <Play className="h-3 w-3" />
              {status === "starting" ? "Starting..." : "Start"}
            </Button>
          )}
        </div>
      </CardContent>
    </Card>
  )
}
