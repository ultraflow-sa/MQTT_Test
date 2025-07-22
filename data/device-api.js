/**
 * Device API - Unified Communication Library
 * Automatically detects ESP32 mode (AP/STA) and uses appropriate communication method
 * Compatible with both ESP32 web interface and server-side applications
 */

class DeviceAPI {
    constructor(config = {}) {
        this.mode = 'unknown';
        this.deviceSerial = null;
        this.deviceVersion = null;
        this.deviceIP = null;
        this.mqttClient = null;
        this.isInitialized = false;
        this.statusUpdateInterval = null;
        this.eventListeners = new Map();
        
        // Configuration options
        this.config = {
            autoDetectMode: true,
            statusUpdateInterval: 2000,
            mqttReconnectDelay: 5000,
            apiTimeout: 5000,
            debug: false,
            ...config
        };
        
        // Event emitter setup
        this.events = new EventTarget();
        
        if (this.config.autoDetectMode) {
            this.init();
        }
    }
    
    /**
     * Initialize the device API - detects mode and sets up communication
     */
    async init() {
        if (this.isInitialized) return this.getStatus();
        
        this.log('Initializing Device API...');
        
        try {
            // Try to detect device mode via web API
            const status = await this.detectDeviceMode();
            
            if (status) {
                this.mode = status.mode;
                this.deviceSerial = status.serial;
                this.deviceVersion = status.version;
                this.deviceIP = status.ip_address;
                this.isInitialized = true;
                
                this.log(`Device API initialized - Mode: ${this.mode}, Serial: ${this.deviceSerial}`);
                
                if (this.mode === 'STA') {
                    await this.initMQTT();
                }
                
                this.startStatusUpdates();
                this.emit('initialized', status);
                
                return status;
            } else {
                throw new Error('Failed to detect device mode');
            }
            
        } catch (error) {
            this.log('Failed to initialize Device API:', error);
            // Fallback mode
            this.mode = 'unknown';
            this.isInitialized = true;
            this.emit('error', { type: 'init_failed', error });
            return null;
        }
    }
    
    /**
     * Detect device mode by checking API availability
     */
    async detectDeviceMode() {
        try {
            const controller = new AbortController();
            const timeoutId = setTimeout(() => controller.abort(), this.config.apiTimeout);
            
            const response = await fetch('/api/status', {
                signal: controller.signal,
                headers: { 'Accept': 'application/json' }
            });
            
            clearTimeout(timeoutId);
            
            if (response.ok) {
                const status = await response.json();
                this.log('Device mode detected:', status);
                return status;
            } else {
                throw new Error(`API returned ${response.status}`);
            }
        } catch (error) {
            this.log('Device mode detection failed:', error);
            return null;
        }
    }
    
    /**
     * Initialize MQTT connection for STA mode
     */
    async initMQTT() {
        if (typeof mqtt === 'undefined') {
            this.log('MQTT library not available');
            return false;
        }
        
        try {
            // Use existing MQTT client if available (from main.html)
            if (typeof client !== 'undefined' && client && client.connected) {
                this.mqttClient = client;
                this.log('Using existing MQTT connection');
                return true;
            }
            
            // Create new MQTT client if needed
            this.log('Creating new MQTT connection...');
            // Note: You'd need to configure your MQTT broker details here
            // For now, we'll rely on the existing connection from main.html
            
        } catch (error) {
            this.log('MQTT initialization failed:', error);
            return false;
        }
        
        return false;
    }
    
    /**
     * Get current device status
     */
    async getStatus() {
        try {
            const response = await fetch('/api/status');
            if (response.ok) {
                const status = await response.json();
                this.emit('statusUpdate', status);
                return status;
            }
        } catch (error) {
            this.log('Failed to get status:', error);
        }
        return null;
    }
    
    /**
     * Control pump (unified method for both modes)
     */
    async controlPump(pumpNumber, action) {
        const result = { success: false, method: this.mode, action, pumpNumber };
        
        if (this.mode === 'STA' && this.mqttClient && this.mqttClient.connected) {
            // Use MQTT in STA mode
            try {
                const topic = `a3/${this.deviceSerial}/test/pump${pumpNumber}`;
                this.mqttClient.publish(topic, action);
                result.success = true;
                result.method = 'mqtt';
                this.log(`Pump${pumpNumber} ${action} via MQTT`);
            } catch (error) {
                result.error = error.message;
                this.log('MQTT pump control failed:', error);
            }
        } else {
            // Use web API in AP mode or as fallback
            try {
                const response = await fetch(`/api/pump${pumpNumber}/control`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                    body: `action=${action}`
                });
                
                if (response.ok) {
                    const data = await response.json();
                    result.success = true;
                    result.method = 'web';
                    result.response = data;
                    this.log(`Pump${pumpNumber} ${action} via Web API`);
                } else {
                    result.error = `HTTP ${response.status}`;
                }
            } catch (error) {
                result.error = error.message;
                this.log('Web API pump control failed:', error);
            }
        }
        
        this.emit('pumpControl', result);
        return result;
    }
    
    /**
     * Get device settings
     */
    async getSettings(settingsType = 'all') {
        const result = { success: false, method: this.mode, settingsType };
        
        if (this.mode === 'STA' && this.mqttClient && this.mqttClient.connected) {
            // In STA mode, rely on existing MQTT logic from main.html
            result.success = false;
            result.method = 'mqtt';
            result.message = 'Use existing MQTT settings query from main.html';
            return result;
        } else {
            // Use web API in AP mode
            try {
                if (settingsType === 'all') {
                    // Get all settings in parallel
                    const [p1, p2, extra] = await Promise.all([
                        fetch('/api/settings/p1').then(r => r.json()),
                        fetch('/api/settings/p2').then(r => r.json()),
                        fetch('/api/settings/extra').then(r => r.json())
                    ]);
                    
                    result.success = true;
                    result.method = 'web';
                    result.data = { p1, p2, extra };
                } else {
                    // Get specific settings type
                    const response = await fetch(`/api/settings/${settingsType}`);
                    if (response.ok) {
                        result.success = true;
                        result.method = 'web';
                        result.data = await response.json();
                    } else {
                        result.error = `HTTP ${response.status}`;
                    }
                }
            } catch (error) {
                result.error = error.message;
                this.log('Web API settings get failed:', error);
            }
        }
        
        this.emit('settingsGet', result);
        return result;
    }
    
    /**
     * Save device settings
     */
    async saveSettings(settingsType, settings) {
        const result = { success: false, method: this.mode, settingsType };
        
        if (this.mode === 'STA' && this.mqttClient && this.mqttClient.connected) {
            // Use MQTT in STA mode
            try {
                const topic = `a3/${this.deviceSerial}/${settingsType}settingsSave`;
                this.mqttClient.publish(topic, JSON.stringify(settings));
                result.success = true;
                result.method = 'mqtt';
                this.log(`Settings ${settingsType} saved via MQTT`);
            } catch (error) {
                result.error = error.message;
                this.log('MQTT settings save failed:', error);
            }
        } else {
            // Use web API in AP mode
            try {
                const response = await fetch(`/api/settings/${settingsType}`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(settings)
                });
                
                if (response.ok) {
                    const data = await response.json();
                    result.success = true;
                    result.method = 'web';
                    result.response = data;
                    this.log(`Settings ${settingsType} saved via Web API`);
                } else {
                    result.error = `HTTP ${response.status}`;
                }
            } catch (error) {
                result.error = error.message;
                this.log('Web API settings save failed:', error);
            }
        }
        
        this.emit('settingsSave', result);
        return result;
    }
    
    /**
     * Get WiFi configuration
     */
    async getWiFiConfig() {
        try {
            const response = await fetch('/api/wifi');
            if (response.ok) {
                return await response.json();
            }
        } catch (error) {
            this.log('Failed to get WiFi config:', error);
        }
        return null;
    }
    
    /**
     * Save WiFi configuration
     */
    async saveWiFiConfig(ssid, password) {
        const result = { success: false };
        
        try {
            const response = await fetch('/savewifi', {
                method: 'POST',
                headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
                body: `ssid=${encodeURIComponent(ssid)}&password=${encodeURIComponent(password)}`
            });
            
            if (response.ok) {
                result.success = true;
                result.message = await response.text();
                this.log('WiFi config saved');
            } else {
                result.error = `HTTP ${response.status}`;
            }
        } catch (error) {
            result.error = error.message;
            this.log('WiFi config save failed:', error);
        }
        
        this.emit('wifiSave', result);
        return result;
    }
    
    /**
     * Start periodic status updates
     */
    startStatusUpdates() {
        if (this.statusUpdateInterval) {
            clearInterval(this.statusUpdateInterval);
        }
        
        this.statusUpdateInterval = setInterval(async () => {
            await this.getStatus();
        }, this.config.statusUpdateInterval);
        
        this.log('Status updates started');
    }
    
    /**
     * Stop status updates
     */
    stopStatusUpdates() {
        if (this.statusUpdateInterval) {
            clearInterval(this.statusUpdateInterval);
            this.statusUpdateInterval = null;
            this.log('Status updates stopped');
        }
    }
    
    /**
     * Event system
     */
    on(eventType, callback) {
        if (!this.eventListeners.has(eventType)) {
            this.eventListeners.set(eventType, []);
        }
        this.eventListeners.get(eventType).push(callback);
        
        // Also listen on the EventTarget for compatibility
        this.events.addEventListener(eventType, callback);
    }
    
    off(eventType, callback) {
        if (this.eventListeners.has(eventType)) {
            const listeners = this.eventListeners.get(eventType);
            const index = listeners.indexOf(callback);
            if (index > -1) {
                listeners.splice(index, 1);
            }
        }
        this.events.removeEventListener(eventType, callback);
    }
    
    emit(eventType, data) {
        // Emit via EventTarget
        this.events.dispatchEvent(new CustomEvent(eventType, { detail: data }));
        
        // Emit via manual listeners
        if (this.eventListeners.has(eventType)) {
            this.eventListeners.get(eventType).forEach(callback => {
                try {
                    callback(data);
                } catch (error) {
                    this.log('Event callback error:', error);
                }
            });
        }
    }
    
    /**
     * Cleanup resources
     */
    destroy() {
        this.stopStatusUpdates();
        this.eventListeners.clear();
        this.isInitialized = false;
        this.log('Device API destroyed');
    }
    
    /**
     * Debug logging
     */
    log(...args) {
        if (this.config.debug) {
            console.log('[DeviceAPI]', ...args);
        }
    }
    
    /**
     * Static method to create and initialize API
     */
    static async create(config = {}) {
        const api = new DeviceAPI(config);
        await api.init();
        return api;
    }
}

// Export for both browser and Node.js environments
if (typeof module !== 'undefined' && module.exports) {
    // Node.js environment
    module.exports = DeviceAPI;
} else if (typeof window !== 'undefined') {
    // Browser environment - just make the class available globally
    window.DeviceAPI = DeviceAPI;
    
    // Only auto-initialize on local hosting
    const hostname = window.location.hostname;
    const isLocalHost = hostname === 'localhost' || 
                       hostname.startsWith('192.168.') || 
                       hostname.startsWith('10.') || 
                       hostname.startsWith('172.') ||
                       hostname.endsWith('.local') ||
                       /^\d+\.\d+\.\d+\.\d+$/.test(hostname);
    
    if (isLocalHost) {
        console.log('[DeviceAPI] Local hosting detected - auto-initializing');
        // Auto-initialize on local hosting only
        if (document.readyState === 'loading') {
            document.addEventListener('DOMContentLoaded', () => {
                window.deviceAPI = new DeviceAPI({ debug: true });
            });
        } else {
            window.deviceAPI = new DeviceAPI({ debug: true });
        }
    } else {
        console.log('[DeviceAPI] Remote hosting detected - manual initialization only');
        // Don't auto-initialize on remote hosting
        window.deviceAPI = null;
    }
}

class UnifiedDeviceConnection {
  constructor() {
    this.connectionType = null;
    this.mqttClient = null;
    this.ws = null;
    this.deviceSerial = null;
    this.isConnected = false;
    this.heartbeatInterval = null;
  }

  async init() {
    console.log("Initializing unified connection...");
    
    // Detect connection type based on URL
    if (window.location.hostname.includes('.local') || 
        window.location.hostname.startsWith('192.168.4.') ||
        window.location.hostname.startsWith('192.168.1.')) {
      // Local connection (BLE mode)
      this.connectionType = 'local';
      await this.initLocalConnection();
    } else {
      // Cloud connection (WiFi mode)
      this.connectionType = 'cloud';
      await this.initCloudConnection();
    }
  }

  async initLocalConnection() {
    console.log("Initializing local connection (BLE mode)");
    
    try {
      // Connect via WebSocket to local ESP32
      this.ws = new WebSocket(`ws://${window.location.host}/ws`);
      
      this.ws.onopen = () => {
        console.log("Local WebSocket connected");
        this.isConnected = true;
        this.startHeartbeat();
        this.onConnectionEstablished();
      };
      
      this.ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          this.handleMessage(data.topic, data.payload);
        } catch (e) {
          console.error('WebSocket message parse error:', e);
        }
      };
      
      this.ws.onclose = () => {
        console.log("Local WebSocket disconnected");
        this.isConnected = false;
        this.stopHeartbeat();
        this.onConnectionLost();
      };
      
    } catch (error) {
      console.error("Local connection failed:", error);
    }
  }

  async initCloudConnection() {
    console.log("Initializing cloud connection (WiFi mode)");
    
    try {
      // Connect to AWS-hosted MQTT broker
      this.mqttClient = new Paho.MQTT.Client(
        "your-aws-mqtt-broker.com", 
        443, 
        "web_client_" + Math.random()
      );
      
      this.mqttClient.onConnectionLost = (responseObject) => {
        console.log("Cloud MQTT connection lost:", responseObject.errorMessage);
        this.isConnected = false;
        this.stopHeartbeat();
        this.onConnectionLost();
      };
      
      this.mqttClient.onMessageArrived = (message) => {
        this.handleMessage(message.destinationName, message.payloadString);
      };
      
      await this.mqttClient.connect({
        onSuccess: () => {
          console.log("Cloud MQTT connected");
          this.isConnected = true;
          this.startHeartbeat();
          this.onConnectionEstablished();
        },
        onFailure: (error) => {
          console.error("Cloud MQTT connection failed:", error);
        }
      });
      
    } catch (error) {
      console.error("Cloud connection failed:", error);
    }
  }

  sendMessage(topic, payload) {
    if (!this.isConnected) {
      console.error("Not connected - cannot send message");
      return;
    }
    
    if (this.connectionType === 'local') {
      // Send via WebSocket
      const message = {
        topic: topic,
        payload: payload,
        timestamp: Date.now()
      };
      this.ws.send(JSON.stringify(message));
    } else {
      // Send via MQTT
      const message = new Paho.MQTT.Message(payload);
      message.destinationName = topic;
      this.mqttClient.send(message);
    }
  }

  startHeartbeat() {
    this.heartbeatInterval = setInterval(() => {
      if (this.deviceSerial) {
        this.sendMessage(`a3/${this.deviceSerial}/webHeartbeat`, "ping");
      }
    }, 10000); // Every 10 seconds
  }

  stopHeartbeat() {
    if (this.heartbeatInterval) {
      clearInterval(this.heartbeatInterval);
      this.heartbeatInterval = null;
    }
  }

  handleMessage(topic, payload) {
    // Your existing message handling logic
    console.log(`Received: ${topic} -> ${payload}`);
    
    // Update UI based on message
    this.updateUI(topic, payload);
  }

  onConnectionEstablished() {
    // Show connected status in UI
    document.getElementById('connectionStatus').textContent = 
      `Connected via ${this.connectionType === 'local' ? 'BLE' : 'WiFi'}`;
    document.getElementById('connectionStatus').className = 'connected';
    
    // Request device serial number
    this.sendMessage("a3/querySerial", "");
  }

  onConnectionLost() {
    // Show disconnected status in UI
    document.getElementById('connectionStatus').textContent = 'Disconnected';
    document.getElementById('connectionStatus').className = 'disconnected';
  }

  updateUI(topic, payload) {
    // Your existing UI update logic
    // This remains the same regardless of connection type
  }
}

// Initialize on page load
let deviceConnection;
window.addEventListener('load', async () => {
  deviceConnection = new UnifiedDeviceConnection();
  await deviceConnection.init();
});