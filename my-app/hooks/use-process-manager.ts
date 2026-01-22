import { useState, useEffect } from "react"

interface ProcessState {
  status: "running" | "stopped" | "starting" | "stopping" | "error"
  pid?: number
  uptime?: number
  command?: string
}

interface ProcessManager {
  processes: Record<string, ProcessState>
  loading: boolean
  error: string | null
  startProcess: (processKey: string) => Promise<void>
  stopProcess: (processKey: string) => Promise<void>
  startAll: () => Promise<void>
  stopAll: () => Promise<void>
  emergencyStop: () => Promise<void>
  refresh: () => Promise<void>
}

export function useProcessManager(): ProcessManager {
  const [processes, setProcesses] = useState<Record<string, ProcessState>>({})
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)

  const fetchProcesses = async () => {
    try {
      const response = await fetch("/api/launch")
      const data = await response.json()

      if (response.ok) {
        // Convert API response to ProcessState format
        const processStates: Record<string, ProcessState> = {}
        Object.entries(data.processes || {}).forEach(([key, value]: [string, any]) => {
          processStates[key] = {
            status: value.status || 'stopped',
            pid: value.pid
          }
        })
        setProcesses(processStates)
        setError(null)
      } else {
        throw new Error(data.error || "Failed to fetch processes")
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : "Unknown error")
    } finally {
      setLoading(false)
    }
  }

  const startProcess = async (processKey: string) => {
    try {
      setProcesses(prev => ({
        ...prev,
        [processKey]: { ...prev[processKey], status: "starting" }
      }))

      const response = await fetch("/api/launch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: "start", launchKey: processKey })
      })

      const result = await response.json()

      if (!response.ok) {
        throw new Error(result.error)
      }

      // Immediately set as running with PID
      setProcesses(prev => ({
        ...prev,
        [processKey]: { status: "running", pid: result.pid }
      }))

      // Refresh state after a short delay to confirm
      setTimeout(() => fetchProcesses(), 1000)
      setTimeout(() => fetchProcesses(), 3000)
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to start process")
      setProcesses(prev => ({
        ...prev,
        [processKey]: { ...prev[processKey], status: "error" }
      }))
    }
  }

  const stopProcess = async (processKey: string) => {
    try {
      // Set stopping status immediately
      setProcesses(prev => ({
        ...prev,
        [processKey]: { ...prev[processKey], status: "stopping" }
      }))

      const response = await fetch("/api/launch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: "stop", launchKey: processKey })
      })

      const result = await response.json()

      if (!response.ok) {
        throw new Error(result.error)
      }

      // Set as stopped
      setProcesses(prev => ({
        ...prev,
        [processKey]: { status: "stopped" }
      }))

      // Refresh after stop completes
      await fetchProcesses()
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to stop process")
      setProcesses(prev => ({
        ...prev,
        [processKey]: { ...prev[processKey], status: "error" }
      }))
    }
  }

  const startAll = async () => {
    try {
      // Start all processes sequentially
      const processKeys = Object.keys(processes)
      for (const key of processKeys) {
        await startProcess(key)
        // Small delay between starts
        await new Promise(resolve => setTimeout(resolve, 500))
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to start all processes")
    }
  }

  const stopAll = async () => {
    try {
      const response = await fetch("/api/launch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: "stop-all" })
      })

      const result = await response.json()

      if (!response.ok) {
        throw new Error(result.error)
      }

      // Clear all local state
      setProcesses({})
      await fetchProcesses()
    } catch (err) {
      setError(err instanceof Error ? err.message : "Failed to stop all processes")
    }
  }

  const emergencyStop = async () => {
    try {
      const response = await fetch("/api/launch", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ action: "emergency-stop" })
      })

      const result = await response.json()

      if (!response.ok) {
        throw new Error(result.error)
      }

      // Clear all local state immediately
      setProcesses({})
      await fetchProcesses()
    } catch (err) {
      setError(err instanceof Error ? err.message : "Emergency stop failed")
    }
  }

  const refresh = async () => {
    setLoading(true)
    await fetchProcesses()
  }

  // Initial load
  useEffect(() => {
    fetchProcesses()
  }, [])

  // Auto-refresh every 3 seconds for faster state updates
  useEffect(() => {
    const interval = setInterval(fetchProcesses, 3000)
    return () => clearInterval(interval)
  }, [])

  return {
    processes,
    loading,
    error,
    startProcess,
    stopProcess,
    startAll,
    stopAll,
    emergencyStop,
    refresh
  }
}