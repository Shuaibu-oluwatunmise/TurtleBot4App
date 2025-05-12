import React, { useState, useRef, useEffect } from 'react';
import {
  View,
  Text,
  StyleSheet,
  TextInput,
  TouchableOpacity,
  SafeAreaView,
  Modal,
} from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import HeaderBar from '@/components/HeaderBar';
import ButtonControl from '@/components/ButtonControl';
import JoystickControl from '@/components/JoystickControl';
import MapCanvas from '@/components/MapCanvas'; 
import debounce from 'lodash.debounce';
import { useCallback } from 'react';


export default function MapScreen() {
  const [mappingMode, setMappingMode] = useState<'auto' | 'manual'>('manual');
  const [controlMode, setControlMode] = useState<'buttons' | 'joystick'>('buttons');
  const [mapName, setMapName] = useState('my_map');
  const [savedStatus, setSavedStatus] = useState('');
  const [showModeDropdown, setShowModeDropdown] = useState(false);
  const [showControlDropdown, setShowControlDropdown] = useState(false);
  const [robotIp, setRobotIp] = useState<string | null>(null);
  const ws = useRef<WebSocket | null>(null);
  const [mapData, setMapData] = useState(null);
  const [isMappingActive, setIsMappingActive] = useState(true);
  const lastMapUpdate = useRef(Date.now());
  const mapSocket = useRef<WebSocket | null>(null);
  const controlSocket = useRef<WebSocket | null>(null);


  useEffect(() => {
    const setupSockets = async () => {
        const ip = await AsyncStorage.getItem('robotIp');
        if (!ip) return;

        // MAP SOCKET
        const mapWS = new WebSocket(`ws://${ip}:9090`);
        mapSocket.current = mapWS;

        mapWS.onopen = () => {
        console.log('🧭 Map WebSocket connected');
        mapWS.send(JSON.stringify({
            op: 'subscribe',
            topic: '/map',
            type: 'nav_msgs/OccupancyGrid',
        }));
        };

        mapWS.onmessage = (event) => {
        const now = Date.now();
        if (now - lastMapUpdate.current < 500) return;
        lastMapUpdate.current = now;
        try {
            const message = JSON.parse(event.data);
            if (message?.msg?.info && message?.msg?.data) {
            setMapData(message.msg);
            }
        } catch (err) {
            console.error('❌ Map WS error:', err);
        }
        };

        mapWS.onerror = (e) => console.error('Map socket error:', e.message);

        // CONTROL SOCKET
        const ctrlWS = new WebSocket(`ws://${ip}:9090`);
        controlSocket.current = ctrlWS;

        ctrlWS.onopen = () => console.log('🎮 Control WebSocket connected');
        ctrlWS.onerror = (e) => console.error('Control socket error:', e.message);
    };

    setupSockets();
    return () => {
        mapSocket.current?.close();
        controlSocket.current?.close();
    };
    }, []); 

  const handleStartMapping = () => {
    console.log(`🚀 Starting ${mappingMode} mapping`);
    setIsMappingActive(true);
  };

  const handleStopMapping = () => {
    console.log('🛑 Stopping mapping');
    setIsMappingActive(false);
  };

  const handleSaveMap = () => {
    if (!mapName) return;
    console.log(`💾 Saving map as: ${mapName}`);
    setSavedStatus(`Map saved as ${mapName}.pgm / .yaml`);
  };

  const rawSendCommand = (linear: number, angular: number) => {
    if (!isMappingActive) return;
    if (!controlSocket.current || controlSocket.current.readyState !== WebSocket.OPEN) return;

    const cmd = {
        op: 'publish',
        topic: '/cmd_vel',
        msg: {
        header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
        twist: {
            linear: { x: linear, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: angular },
        },
        },
    };

    console.log("🚀 Sending command:", {
        linear,
        angular,
        wsState: controlSocket.current.readyState,
    });

    controlSocket.current.send(JSON.stringify(cmd));
    };

    // Debounce with 100ms delay (adjust as needed)
    const sendCommand = useCallback(
    debounce(rawSendCommand, 100, { leading: true, trailing: true }),
    [isMappingActive, ws.current]
    );

  return (
    <SafeAreaView style={styles.safeArea}>
      <HeaderBar title="🗺️ Mapping Station" ws={ws.current} onRefresh={() => {}} />
      <View style={styles.container}>
        <View style={styles.topRow}>
          <View style={styles.modeSection}>
            <Text style={styles.label}>Mode</Text>
            <TouchableOpacity
              style={styles.dropdownButton}
              onPress={() => setShowModeDropdown(true)}
            >
              <Text style={styles.dropdownText}>
                {mappingMode === 'manual' ? 'Manual' : 'Auto'} ▼
              </Text>
            </TouchableOpacity>
          </View>

          <View style={styles.nameSection}>
            <Text style={styles.label}>Map Name</Text>
            <TextInput
              style={styles.input}
              value={mapName}
              onChangeText={setMapName}
              placeholder="map_name"
              placeholderTextColor="#888"
            />
          </View>
        </View>

        <Modal visible={showModeDropdown} transparent animationType="fade">
          <TouchableOpacity
            style={styles.modalOverlay}
            onPress={() => setShowModeDropdown(false)}
          >
            <View style={styles.modalContent}>
              {['manual', 'auto'].map((mode) => (
                <TouchableOpacity
                  key={mode}
                  style={styles.modalItem}
                  onPress={() => {
                    setMappingMode(mode as 'manual' | 'auto');
                    setShowModeDropdown(false);
                  }}
                >
                  <Text style={styles.modalItemText}>
                    {mode === 'manual' ? 'Manual' : 'Auto'}
                  </Text>
                </TouchableOpacity>
              ))}
            </View>
          </TouchableOpacity>
        </Modal>

        <View style={styles.mapViewBox}>
          <MapCanvas map={mapData} />
        </View>

        {mappingMode === 'manual' ? (
          <>
            <View style={styles.controlDropdownWrapper}>
              <Text style={styles.label}>Control Type</Text>
              <TouchableOpacity
                style={[styles.dropdownButton, { width: 180 }]}
                onPress={() => setShowControlDropdown(true)}
              >
                <Text style={styles.dropdownText}>
                  {controlMode === 'buttons' ? 'Button Control' : 'Joystick Control'} ▼
                </Text>
              </TouchableOpacity>
            </View>

            <Modal visible={showControlDropdown} transparent animationType="fade">
              <TouchableOpacity
                style={styles.modalOverlay}
                onPress={() => setShowControlDropdown(false)}
              >
                <View style={styles.modalContent}>
                  {['buttons', 'joystick'].map((mode) => (
                    <TouchableOpacity
                      key={mode}
                      style={styles.modalItem}
                      onPress={() => {
                        setControlMode(mode as 'buttons' | 'joystick');
                        setShowControlDropdown(false);
                      }}
                    >
                      <Text style={styles.modalItemText}>
                        {mode === 'buttons' ? 'Button Control' : 'Joystick Control'}
                      </Text>
                    </TouchableOpacity>
                  ))}
                </View>
              </TouchableOpacity>
            </Modal>

            <View style={styles.manualControlArea}>
              {controlMode === 'buttons' ? (
                <ButtonControl
                  linearSpeed={0.5}
                  angularSpeed={1.0}
                  sendCommand={sendCommand}
                  enabled={true}
                />
              ) : (
                <JoystickControl
                  ws={ws}
                  linearSpeed={0.5}
                  angularSpeed={1.0}
                  enabled={true}
                />
              )}
            </View>
          </>
        ) : (
          <View style={styles.buttonRow}>
            <TouchableOpacity style={styles.controlButton} onPress={handleStartMapping}>
              <Text style={styles.controlButtonText}>Start</Text>
            </TouchableOpacity>
            <TouchableOpacity style={styles.controlButton} onPress={handleStopMapping}>
              <Text style={styles.controlButtonText}>Stop</Text>
            </TouchableOpacity>
          </View>
        )}

        <TouchableOpacity style={styles.saveButton} onPress={handleSaveMap}>
          <Text style={styles.saveButtonText}>Save Map</Text>
        </TouchableOpacity>

        {savedStatus && <Text style={styles.savedStatus}>{savedStatus}</Text>}
      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  safeArea: {
    flex: 1,
    backgroundColor: '#1e1e1e',
  },
  container: {
    padding: 20,
    alignItems: 'center',
  },
  topRow: {
    flexDirection: 'row',
    width: '100%',
    justifyContent: 'space-between',
  },
  modeSection: {
    flex: 1,
    marginRight: 10,
  },
  nameSection: {
    flex: 2,
  },
  controlDropdownWrapper: {
    marginTop: 20,
    width: '100%',
    alignItems: 'flex-start',
  },
  label: {
    color: '#00ffcc',
    fontSize: 14,
    marginBottom: 4,
  },
  dropdownButton: {
    backgroundColor: '#333',
    paddingVertical: 10,
    paddingHorizontal: 12,
    borderRadius: 10,
  },
  dropdownText: {
    color: '#fff',
  },
  input: {
    backgroundColor: '#2a2a2a',
    color: '#fff',
    borderRadius: 10,
    borderColor: '#00ffcc',
    borderWidth: 1,
    padding: 10,
  },
  mapViewBox: {
    backgroundColor: '#2a2a2a',
    borderRadius: 12,
    height: 200,
    width: '100%',
    marginTop: 20,
    overflow: 'hidden',
    borderColor: '#00ffcc',
    borderWidth: 1,
  },
  buttonRow: {
    flexDirection: 'row',
    justifyContent: 'space-evenly',
    width: '100%',
    marginVertical: 20,
  },
  controlButton: {
    backgroundColor: '#444',
    paddingVertical: 12,
    paddingHorizontal: 24,
    borderRadius: 10,
  },
  controlButtonText: {
    color: '#fff',
    fontWeight: '600',
  },
  saveButton: {
    backgroundColor: '#00ffcc',
    paddingVertical: 14,
    paddingHorizontal: 30,
    borderRadius: 12,
    marginTop: 20,
  },
  saveButtonText: {
    color: '#000',
    fontWeight: 'bold',
    fontSize: 16,
  },
  savedStatus: {
    color: '#0f0',
    marginTop: 12,
    fontSize: 14,
  },
  manualControlArea: {
    marginTop: 20,
    width: '100%',
  },
  modalOverlay: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    backgroundColor: 'rgba(0,0,0,0.6)',
  },
  modalContent: {
    backgroundColor: '#2c2c2c',
    padding: 20,
    borderRadius: 12,
    width: '80%',
  },
  modalItem: {
    paddingVertical: 10,
  },
  modalItemText: {
    color: '#00ffcc',
    fontSize: 16,
    textAlign: 'center',
  },
});