"use client"

import { useState, useEffect } from "react"
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card"
import { Button } from "@/components/ui/button"
import { Badge } from "@/components/ui/badge"
import { ScrollArea } from "@/components/ui/scroll-area"
import { Textarea } from "@/components/ui/textarea"
import { FileText, Trash2, Download } from "lucide-react"

interface LogEntry {
  timestamp: string
  level: "info" | "warn" | "error" | "debug"
  message: string
  source?: string
}

export function LogViewer() {
  const [logs, setLogs] = useState<LogEntry[]>([])
  const [loading, setLoading] = useState(false)

  const fetchLogs = async () => {
    setLoading(true)
    try {
      const response = await fetch("/api/logs")
      const data = await response.json()

      if (response.ok && data.logs) {
        setLogs(data.logs)
      }
    } catch (error) {
      console.error("Failed to fetch logs:", error)
    } finally {
      setLoading(false)
    }
  }

  const clearLogs = async () => {
    try {
      // Call API to delete log files
      const response = await fetch("/api/logs", {
        method: "DELETE"
      })

      if (response.ok) {
        setLogs([])
      }
    } catch (error) {
      console.error("Failed to clear logs:", error)
    }
  }

  const downloadLogs = () => {
    const logText = logs.map(log =>
      `[${log.timestamp}] ${log.level.toUpperCase()}: ${log.message}`
    ).join('\n')

    const blob = new Blob([logText], { type: 'text/plain' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `rover-logs-${new Date().toISOString().split('T')[0]}.txt`
    document.body.appendChild(a)
    a.click()
    document.body.removeChild(a)
    URL.revokeObjectURL(url)
  }

  const getLevelColor = (level: string) => {
    switch (level) {
      case "error": return "text-red-500"
      case "warn": return "text-yellow-500"
      case "info": return "text-blue-500"
      case "debug": return "text-gray-500"
      default: return "text-foreground"
    }
  }

  useEffect(() => {
    fetchLogs()
    // Refresh logs every 5 seconds
    const interval = setInterval(fetchLogs, 5000)
    return () => clearInterval(interval)
  }, [])

  return (
    <Card>
      <CardHeader>
        <div className="flex items-center justify-between">
          <div>
            <CardTitle className="flex items-center gap-2">
              <FileText className="h-5 w-5" />
              System Logs
            </CardTitle>
            <CardDescription>
              Real-time logs from ROS processes and rover operations
            </CardDescription>
          </div>
          <div className="flex items-center gap-2">
            <Badge variant="outline">{logs.length} entries</Badge>
            <Button variant="outline" size="sm" onClick={fetchLogs} disabled={loading}>
              Refresh
            </Button>
            <Button variant="outline" size="sm" onClick={downloadLogs} disabled={logs.length === 0}>
              <Download className="h-4 w-4" />
            </Button>
            <Button variant="outline" size="sm" onClick={clearLogs} disabled={logs.length === 0}>
              <Trash2 className="h-4 w-4" />
            </Button>
          </div>
        </div>
      </CardHeader>
      <CardContent>
        <ScrollArea className="h-96 w-full rounded-md border p-4">
          {loading ? (
            <div className="text-center text-muted-foreground py-8">
              Loading logs...
            </div>
          ) : logs.length === 0 ? (
            <div className="text-center text-muted-foreground py-8">
              No logs available. Launch a process to see logs.
            </div>
          ) : (
            <div className="space-y-2">
              {logs.map((log, index) => (
                <div key={index} className="flex gap-2 text-sm font-mono">
                  <span className="text-muted-foreground whitespace-nowrap">
                    [{log.timestamp}]
                  </span>
                  <span className={`font-medium uppercase min-w-[50px] ${getLevelColor(log.level)}`}>
                    {log.level}:
                  </span>
                  <span className="flex-1">{log.message}</span>
                  {log.source && (
                    <Badge variant="outline" className="text-xs">
                      {log.source}
                    </Badge>
                  )}
                </div>
              ))}
            </div>
          )}
        </ScrollArea>
      </CardContent>
    </Card>
  )
}