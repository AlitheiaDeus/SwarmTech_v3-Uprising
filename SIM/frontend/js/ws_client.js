/**
 * SwarmTech3_UpRising: WebSocket Real-Time Telemetry Client (ws_client.js)
 * Establishes duplex communication with the backend /ws endpoint,
 * auto-reconnects on drop, and dispatches telemetry frames to listeners.
 */

class SwarmWSClient {
    constructor() {
        this.socket = null;
        this.isConnected = false;
        this.reconnectAttempts = 0;
        this.maxReconnectDelay = 3000;
        this.listeners = {
            telemetry: [],
            status: [],
            command_ack: [],
            hydration: [],
        };

        this.init();
    }

    init() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const host = window.location.host || 'localhost:8080';
        this.wsUrl = `${protocol}//${host}/ws`;
        this.connect();
    }

    connect() {
        try {
            console.log(`[WS] Connecting to telemetry bridge: ${this.wsUrl}`);
            this.socket = new WebSocket(this.wsUrl);

            this.socket.onopen = () => {
                console.log('[WS] Connected to telemetry server.');
                this.isConnected = true;
                this.reconnectAttempts = 0;
                this._emit('status', true);
            };

            this.socket.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    if (data.type === 'SESSION_HYDRATION') {
                        this._emit('hydration', data);
                    } else if (data.type === 'TELEMETRY_FRAME' || data.sim_time !== undefined) {
                        this._emit('telemetry', data);
                    } else if (data.type === 'COMMAND_ACK') {
                        this._emit('command_ack', data);
                    }
                } catch (e) {
                    console.error('[WS] Failed to parse message JSON:', e);
                }
            };

            this.socket.onclose = () => {
                this.isConnected = false;
                this._emit('status', false);
                this._scheduleReconnect();
            };

            this.socket.onerror = (err) => {
                console.warn('[WS] Socket error:', err);
                this.isConnected = false;
                this._emit('status', false);
            };
        } catch (e) {
            console.error('[WS] Connection attempt failed:', e);
            this._scheduleReconnect();
        }
    }

    _scheduleReconnect() {
        this.reconnectAttempts++;
        const delay = Math.min(1000 * Math.pow(1.5, this.reconnectAttempts), this.maxReconnectDelay);
        console.log(`[WS] Reconnecting in ${Math.round(delay)}ms...`);
        setTimeout(() => this.connect(), delay);
    }

    send(commandObj) {
        if (this.isConnected && this.socket && this.socket.readyState === WebSocket.OPEN) {
            this.socket.send(JSON.stringify(commandObj));
            return true;
        } else {
            console.warn('[WS] Cannot send command, socket offline:', commandObj);
            return false;
        }
    }

    on(event, callback) {
        if (this.listeners[event]) {
            this.listeners[event].push(callback);
        }
    }

    _emit(event, data) {
        if (this.listeners[event]) {
            for (const cb of this.listeners[event]) {
                try {
                    cb(data);
                } catch (e) {
                    console.error(`[WS] Error in listener for ${event}:`, e);
                }
            }
        }
    }
}

// Global telemetry client instance
window.swarmClient = new SwarmWSClient();
