import { NextResponse } from 'next/server'
import * as fs from 'fs'
import * as path from 'path'
import { promisify } from 'util'
import { exec } from 'child_process'

const execAsync = promisify(exec)

const HOME = process.env.HOME || '/home/anish'
const LOG_DIR = path.join(HOME, '.ros/log')
const LAUNCH_LOG_DIR = '/tmp/rover_launch_logs'

interface LogEntry {
    timestamp: string
    level: 'info' | 'warn' | 'error' | 'debug'
    message: string
    source?: string
}

function parseLogLine(logLine: string, source: string): LogEntry | null {
    try {
        if (!logLine.trim() || logLine.length < 5) return null

        // Extract timestamp from [INFO] [timestamp] format or just use the whole line
        let timestamp = new Date().toISOString().slice(11, 19)
        const tsMatch = logLine.match(/\[(\d+\.\d+)\]/) || logLine.match(/\[([^\]]+)\]/)
        if (tsMatch) {
            timestamp = tsMatch[1].slice(0, 15)
        }

        // Determine log level
        let level: 'info' | 'warn' | 'error' | 'debug' = 'info'
        const upperLine = logLine.toUpperCase()
        if (upperLine.includes('[ERROR]') || upperLine.includes('ERROR')) {
            level = 'error'
        } else if (upperLine.includes('[WARN]') || upperLine.includes('WARN')) {
            level = 'warn'
        } else if (upperLine.includes('[DEBUG]')) {
            level = 'debug'
        }

        // Clean up message
        let message = logLine.trim()

        // Skip empty or very short lines
        if (message.length < 3) return null

        return { timestamp, level, message, source }
    } catch {
        return null
    }
}

export async function GET() {
    try {
        const logs: LogEntry[] = []

        // Ensure launch log dir exists
        if (!fs.existsSync(LAUNCH_LOG_DIR)) {
            fs.mkdirSync(LAUNCH_LOG_DIR, { recursive: true })
        }

        // 1. Read from launch output logs (captured by our launch API)
        try {
            const launchLogFiles = fs.readdirSync(LAUNCH_LOG_DIR)
                .filter(f => f.endsWith('.log'))
                .map(f => path.join(LAUNCH_LOG_DIR, f))

            for (const logFile of launchLogFiles) {
                try {
                    const stat = fs.statSync(logFile)
                    // Only read logs from last hour
                    if (Date.now() - stat.mtimeMs > 3600000) continue

                    const content = fs.readFileSync(logFile, 'utf-8')
                    const lines = content.split('\n').slice(-100) // Last 100 lines
                    const source = path.basename(logFile, '.log')

                    for (const line of lines) {
                        const entry = parseLogLine(line, source)
                        if (entry) logs.push(entry)
                    }
                } catch { }
            }
        } catch { }

        // 2. Also read from ROS log files
        try {
            const { stdout } = await execAsync(
                `find ${LOG_DIR} -type f -name '*.log' -mmin -60 2>/dev/null | sort -r | head -20`
            )

            const logFiles = stdout.trim().split('\n').filter(f => f)

            for (const logFile of logFiles) {
                try {
                    const content = fs.readFileSync(logFile, 'utf-8')
                    const lines = content.split('\n').slice(-30)
                    const source = path.basename(logFile, '.log').slice(0, 30)

                    for (const line of lines) {
                        if (line.includes('[')) {
                            const entry = parseLogLine(line, source)
                            if (entry) logs.push(entry)
                        }
                    }
                } catch { }
            }
        } catch { }

        // Sort by timestamp, most recent first
        logs.sort((a, b) => b.timestamp.localeCompare(a.timestamp))

        // Return last 500 logs
        return NextResponse.json({
            logs: logs.slice(0, 500),
            count: logs.length
        })

    } catch (error: any) {
        console.error('Error fetching logs:', error)
        return NextResponse.json({ logs: [], error: error.message, count: 0 })
    }
}

// DELETE endpoint to clear logs
export async function DELETE() {
    try {
        // Clear launch logs
        if (fs.existsSync(LAUNCH_LOG_DIR)) {
            const files = fs.readdirSync(LAUNCH_LOG_DIR)
            for (const file of files) {
                fs.unlinkSync(path.join(LAUNCH_LOG_DIR, file))
            }
        }

        // Clear recent ROS logs
        await execAsync(`find ${LOG_DIR} -type f -name '*.log' -mmin -60 -delete 2>/dev/null || true`)

        return NextResponse.json({ success: true })
    } catch (error: any) {
        return NextResponse.json({ error: error.message }, { status: 500 })
    }
}
