import { exec, spawn } from 'child_process'
import { promisify } from 'util'
import { NextResponse } from 'next/server'
import * as fs from 'fs'
import * as path from 'path'

const execAsync = promisify(exec)

const WORKSPACE = process.env.ROS_WORKSPACE || '/home/anish/test/mars_rover_ros2'

// Process tracking with PIDs stored in file for persistence
const PID_FILE = '/tmp/rover_gui_processes.json'

interface ProcessInfo {
    pid: number
    launchFile: string
    startTime: number
}

function loadProcesses(): Map<string, ProcessInfo> {
    try {
        if (fs.existsSync(PID_FILE)) {
            const data = JSON.parse(fs.readFileSync(PID_FILE, 'utf-8'))
            return new Map(Object.entries(data))
        }
    } catch (error) {
        console.error('Error loading processes:', error)
    }
    return new Map()
}

function saveProcesses(processes: Map<string, ProcessInfo>) {
    try {
        const data = Object.fromEntries(processes)
        fs.writeFileSync(PID_FILE, JSON.stringify(data, null, 2))
    } catch (error) {
        console.error('Error saving processes:', error)
    }
}

async function isProcessRunning(pid: number): Promise<boolean> {
    try {
        await execAsync(`kill -0 ${pid}`)
        return true
    } catch {
        return false
    }
}

export async function POST(request: Request) {
    try {
        const { launchKey, action } = await request.json()
        const processes = loadProcesses()

        if (action === 'start') {
            // Check if already running
            const existing = processes.get(launchKey)
            if (existing && await isProcessRunning(existing.pid)) {
                return NextResponse.json({
                    error: 'Process already running',
                    pid: existing.pid
                }, { status: 400 })
            }

            // Launch file mapping
            const launchFiles: Record<string, string> = {
                hardware: 'hardware.launch.py',
                gazebo_sim: 'gazebo_sim.launch.py',
                complete_system: 'complete_system.launch.py',
                gps_navigation: 'gps_navigation.launch.py',
                dual_ekf_navsat: 'dual_ekf_navsat.launch.py',
                aruco: 'aruco.launch.py',
                mapping: 'mapping.launch.py',
                display: 'display.launch.py',
                controllers: 'controllers.launch.py',
                gz_plus_control: 'gz_plus_control.launch.py',
                foxglove_viz: 'foxglove_viz.launch.py',
                complete_foxglove: 'complete_foxglove.launch.py',
                foxglove_sim: 'foxglove_sim.launch.py'
            }

            const launchFile = launchFiles[launchKey]
            if (!launchFile) {
                return NextResponse.json({ error: 'Unknown launch file' }, { status: 400 })
            }

            // Create a wrapper script that launches in terminal and saves PID + logs
            const pidFile = `/tmp/rover_${launchKey}.pid`
            const logFile = `/tmp/rover_launch_logs/${launchKey}.log`
            const terminalTitle = `${launchFile} - Mars Rover`

            // Ensure log directory exists
            if (!fs.existsSync('/tmp/rover_launch_logs')) {
                fs.mkdirSync('/tmp/rover_launch_logs', { recursive: true })
            }

            // Launch command that saves PID and captures output to log file
            const launchScript = `
#!/bin/bash
source ${WORKSPACE}/install/setup.bash
echo "[$(date)] Starting ${launchFile}..." | tee ${logFile}
ros2 launch rover_description ${launchFile} 2>&1 | tee -a ${logFile} &
LAUNCH_PID=$!
echo $LAUNCH_PID > ${pidFile}
wait $LAUNCH_PID
echo "[$(date)] Process ended" | tee -a ${logFile}
rm -f ${pidFile}
`

            const scriptPath = `/tmp/rover_launch_${launchKey}.sh`
            fs.writeFileSync(scriptPath, launchScript)
            fs.chmodSync(scriptPath, '755')

            // Launch in terminal
            const terminalCmd = `gnome-terminal --title='${terminalTitle}' -- bash -c '${scriptPath}; exec bash'`

            // Execute the terminal launch
            exec(terminalCmd)

            // Wait a bit for the PID file to be created
            await new Promise(resolve => setTimeout(resolve, 1000))

            // Read the actual ROS process PID
            let pid = 0
            try {
                const pidContent = fs.readFileSync(pidFile, 'utf-8').trim()
                pid = parseInt(pidContent)
            } catch {
                return NextResponse.json({
                    error: 'Failed to get process PID'
                }, { status: 500 })
            }

            // Store process info
            processes.set(launchKey, {
                pid,
                launchFile,
                startTime: Date.now()
            })
            saveProcesses(processes)

            return NextResponse.json({
                success: true,
                pid,
                message: `Started ${launchFile} in terminal`
            })

        } else if (action === 'stop') {
            const processInfo = processes.get(launchKey)

            if (!processInfo) {
                return NextResponse.json({ error: 'Process not running' }, { status: 400 })
            }

            try {
                // ===== GRACEFUL SHUTDOWN SEQUENCE =====
                // Step 1: Try to kill gracefully with SIGTERM (allows cleanup)
                console.log(`Stopping ${processInfo.launchFile} gracefully...`)

                const killTreeGracefully = async (pid: number) => {
                    try {
                        // Get all children
                        const { stdout } = await execAsync(`pgrep -P ${pid}`)
                        const childPids = stdout.trim().split('\n').filter(p => p)

                        // Gracefully kill children first
                        for (const childPid of childPids) {
                            await killTreeGracefully(parseInt(childPid))
                        }

                        // Send SIGTERM (graceful shutdown) - but NOT to rosbridge
                        try {
                            const { stdout: cmdline } = await execAsync(`ps -p ${pid} -o cmd=`).catch(() => ({ stdout: '' }))
                            if (!cmdline.includes('rosbridge')) {
                                await execAsync(`kill -TERM ${pid}`)
                            }
                        } catch { }
                    } catch { }
                }

                // Start graceful shutdown
                await killTreeGracefully(processInfo.pid)

                // Also send SIGTERM to process by name (but not rosbridge)
                // This command targets the main ros2 launch process. If rosbridge is a child, it's handled by killTreeGracefully.
                // If rosbridge is a separate launch, it won't be caught here unless launchFile is rosbridge itself.
                // To explicitly exclude rosbridge from this pkill, we'd need a more complex command,
                // but for now, we assume it's handled by the PID-based killTreeGracefully or is a separate process.
                await execAsync(`pkill -TERM -f "ros2 launch rover_description ${processInfo.launchFile}"`).catch(() => { })

                // Step 2: Wait 10 seconds for graceful shutdown and cleanup
                console.log('Waiting 10 seconds for graceful shutdown and cleanup...')
                await new Promise(resolve => setTimeout(resolve, 10000))

                // Step 3: Force kill anything still running (SIGKILL)
                console.log('Force killing remaining processes...')

                const killTreeForce = async (pid: number) => {
                    try {
                        const { stdout } = await execAsync(`pgrep -P ${pid}`)
                        const childPids = stdout.trim().split('\n').filter(p => p)

                        for (const childPid of childPids) {
                            await killTreeForce(parseInt(childPid))
                        }

                        // Force kill with SIGKILL
                        try {
                            await execAsync(`kill -9 ${pid}`)
                        } catch { }
                    } catch { }
                }

                await killTreeForce(processInfo.pid)

                // Force kill by process name
                await execAsync(`pkill -9 -f "ros2 launch rover_description ${processInfo.launchFile}"`).catch(() => { })

                // Step 4: Special handling for Gazebo (it often survives)
                if (processInfo.launchFile.includes('gazebo') || processInfo.launchFile.includes('gz')) {
                    console.log('Killing Gazebo processes...')
                    await execAsync('pkill -9 -f "gz sim"').catch(() => { })
                    await execAsync('killall -9 gz').catch(() => { })
                    await execAsync('killall -9 gzserver').catch(() => { })
                    await execAsync('killall -9 gzclient').catch(() => { })
                }

                console.log(`Successfully stopped ${processInfo.launchFile}`)

                // Remove from tracking
                processes.delete(launchKey)
                saveProcesses(processes)

                // Clean up PID file
                const pidFile = `/tmp/rover_${launchKey}.pid`
                if (fs.existsSync(pidFile)) {
                    fs.unlinkSync(pidFile)
                }

                return NextResponse.json({
                    success: true,
                    message: `Stopped ${processInfo.launchFile}`
                })
            } catch (error) {
                // Process may have already stopped
                processes.delete(launchKey)
                saveProcesses(processes)
                return NextResponse.json({
                    success: true,
                    message: 'Process already stopped'
                })
            }

        } else if (action === 'status') {
            const processInfo = processes.get(launchKey)

            if (!processInfo) {
                return NextResponse.json({ running: false })
            }

            const running = await isProcessRunning(processInfo.pid)
            if (!running) {
                processes.delete(launchKey)
                saveProcesses(processes)
            }

            return NextResponse.json({ running, pid: processInfo.pid })

        } else if (action === 'stop-all' || action === 'emergency-stop') {
            // Stop all tracked processes
            const killPromises = []

            for (const [key, processInfo] of Array.from(processes.entries())) {
                // Kill entire process tree
                const killTree = async (pid: number) => {
                    try {
                        const { stdout } = await execAsync(`pgrep -P ${pid}`)
                        const childPids = stdout.trim().split('\n').filter(p => p)

                        for (const childPid of childPids) {
                            await killTree(parseInt(childPid))
                        }

                        await execAsync(`kill -9 ${pid}`).catch(() => { })
                    } catch { }
                }

                killPromises.push(killTree(processInfo.pid))
            }

            await Promise.all(killPromises)

            // Kill all ros2 launch processes as backup
            await execAsync('pkill -9 -f "ros2 launch"').catch(() => { })

            // Kill all Gazebo processes
            await execAsync('pkill -9 -f "gz sim"').catch(() => { })
            await execAsync('killall -9 gz').catch(() => { })
            await execAsync('killall -9 gzserver').catch(() => { })
            await execAsync('killall -9 gzclient').catch(() => { })

            // Clear all tracking
            processes.clear()
            saveProcesses(processes)

            return NextResponse.json({
                success: true,
                message: 'All processes stopped'
            })
        }

        return NextResponse.json({ error: 'Invalid action' }, { status: 400 })

    } catch (error: any) {
        console.error('Launch API error:', error)
        return NextResponse.json({
            error: error.message || 'Unknown error occurred'
        }, { status: 500 })
    }
}

// GET endpoint to list all launch files
export async function GET() {
    try {
        const processes = loadProcesses()
        const statuses: Record<string, any> = {}

        // Check status of all processes and clean up dead ones
        for (const [key, processInfo] of Array.from(processes.entries())) {
            const running = await isProcessRunning(processInfo.pid)

            if (running) {
                statuses[key] = { status: 'running', pid: processInfo.pid }
            } else {
                processes.delete(key)
                statuses[key] = { status: 'stopped' }
            }
        }

        saveProcesses(processes)

        return NextResponse.json({
            processes: statuses,
            workspace: WORKSPACE
        })
    } catch (error: any) {
        return NextResponse.json({
            error: error.message
        }, { status: 500 })
    }
}
