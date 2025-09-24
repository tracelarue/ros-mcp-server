# Gemini Live with ROS MCP Server

Control ROS robots with voice commands using Google's Gemini Live API.

**Pre-requisites** See the [installation instructions](../../../docs/installation.md) for detailed setup steps.

**Tested In:** Ubuntu 22.04, Python 3.10, ROS2 Humble

## Quick Setup

1. **Install ROS MCP Server**: Follow the [installation guide](../../../docs/installation.md)

2. **Get Google API Key**: Visit [Google AI Studio](https://aistudio.google.com) and create an API key

3. **Create `.env` file**:
   ```env
   GOOGLE_API_KEY="your_google_api_key_here"
   ```

4. **Create `mcp_config.json`**:Replace `/absolute/path/to/ros-mcp-server` with your actual path.
   ```json
   {
      "mcpServers": {
        "ros-mcp-server": {
          "command": "uv",
          "args": [
            "--directory",
            "/home/trace/ros-mcp-server", 
            "run",
            "server.py"
          ]
        }
      }
    }
   ```

## Usage

**Start Gemini Live:**
```bash
cd ros-mcp-server/examples/2_gemini/gemini_live
uv run gemini_live.py --mode=none
```

**Video modes:**
- `--mode=none` - Audio only
- `--mode=camera` - Include camera
- `--mode=screen` - Include screen capture

**Example Voice commands:**
- "Connect to the robot on ip _ and port _ "
- "List all available tools"
- "What ROS topics are available?"
- "Go to the kitchen" 
- "Move forward at 1 m/s for 1 s"

Type `q` + Enter to quit.

## Test with Turtlesim

**Start rosbridge and turtlesim** (separate terminals):
```bash
# Terminal 1: Launch rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Terminal 2: Start turtlesim
ros2 run turtlesim turtlesim_node
```

**Try these voice commands:**
- "Connect to the robot on ip _ and port _ "
- "What ROS topics are available?"
- "Move the turtle forward at 1 m/s and 0 rad/s"
- "Rotate the turtle at 3 rad/s"
- "Change the pen color to red"


See [Turtlesim Tutorial](../../1_turtlesim/README.md) for more examples.

## Troubleshooting

**Not responding to voice?**
- Check microphone permissions and volume
- Test: `arecord -d 5 test.wav && aplay test.wav`

**Robot not moving?**
- Verify robot/simulation is running
- Check rosbridge is running: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
- Check: `ros2 topic list`
- Ask: "List all available tools" to verify MCP connection

**API key errors?**
- Verify `.env` file exists with correct key
- Check key is active in Google AI Studio

**Virtual environment issues?**
- Exit any active environments: `deactivate`
- UV creates its own environment automatically 


