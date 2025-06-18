import React, { useState, useRef, useEffect } from 'react';
import {
  View,
  Text,
  StyleSheet,
  TextInput,
  TouchableOpacity,
  SafeAreaView,
  Modal,
  Keyboard,
  TouchableWithoutFeedback,
} from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import HeaderBar from '@/components/HeaderBar';
import ButtonControl from '@/components/ButtonControl';
import JoystickControl from '@/components/JoystickControl';
import MapViewBox from '@/components/MapViewBox';

export default function MapScreen() {
  const [mappingMode, setMappingMode] = useState<'auto' | 'manual'>('manual');
  const [controlMode, setControlMode] = useState<'buttons' | 'joystick'>('buttons');
  const [mapName, setMapName] = useState('my_map');
  const [savedStatus, setSavedStatus] = useState('');
  const [showModeDropdown, setShowModeDropdown] = useState(false);
  const [showControlDropdown, setShowControlDropdown] = useState(false);
  const [robotIp, setRobotIp] = useState<string | null>(null);
  const [slamRunning, setSlamRunning] = useState(false);
  const [keyboardVisible, setKeyboardVisible] = useState(false);
  const slamSocket = useRef<WebSocket | null>(null);
  const autoMapSocket = useRef<WebSocket | null>(null);
  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    const showSub = Keyboard.addListener('keyboardDidShow', () => setKeyboardVisible(true));
    const hideSub = Keyboard.addListener('keyboardDidHide', () => setKeyboardVisible(false));
    return () => {
      showSub.remove();
      hideSub.remove();
    };
  }, []);

  useEffect(() => {
    if (!robotIp) return;
    slamSocket.current = new WebSocket(`ws://${robotIp}:9099`);
    slamSocket.current.onerror = (err) => {
      console.error('SLAM WebSocket error:', err.message);
    };
    return () => slamSocket.current?.close();
  }, [robotIp]);

  useEffect(() => {
    const fetchIpAndConnect = async () => {
      const ip = await AsyncStorage.getItem('robotIp');
      if (ip) {
        setRobotIp(ip);
        ws.current = new WebSocket(`ws://${ip}:9092`);
        autoMapSocket.current = new WebSocket(`ws://${ip}:9098`);

        autoMapSocket.current.onopen = () => {
          console.log("🟢 Auto mapping WebSocket connected");
        };

        autoMapSocket.current.onmessage = (event) => {
          const data = JSON.parse(event.data);
          if (data.type === 'auto_map_ack') {
            console.log(`✅ Auto mapping status: ${data.status}`);
          }
        };

        autoMapSocket.current.onerror = (err) => {
          console.error("❌ Auto mapping WebSocket error:", err.message);
        };
      }
    };

    fetchIpAndConnect();
    return () => {
      ws.current?.close();
      autoMapSocket.current?.close();
    };
  }, []);

  const toggleSlam = () => {
    if (!slamSocket.current || slamSocket.current.readyState !== WebSocket.OPEN) return;
    slamSocket.current.send(slamRunning ? "stop_slam" : "launch_slam");
    setSlamRunning(!slamRunning);
  };

  const handleStartMapping = () => console.log(`🚀 Starting ${mappingMode} mapping`);
  const handleStopMapping = () => console.log('🛑 Stopping mapping');

  const handleSaveMap = () => {
    if (!mapName || !slamSocket.current || slamSocket.current.readyState !== WebSocket.OPEN) {
      console.warn("⚠️ Cannot save map: socket not ready or map name missing.");
      return;
    }
    slamSocket.current.send(`save_map:${mapName}`);
    setSavedStatus(`📂 Map save requested: ${mapName}`);
    setTimeout(() => setSavedStatus(''), 5000);
  };

  const handleAutoMapping = () => {
    if (!autoMapSocket.current || autoMapSocket.current.readyState !== WebSocket.OPEN) {
      console.warn("⚠️ Cannot start auto mapping: WebSocket not connected.");
      return;
    }

    console.log("📡 Sending Auto_Map command...");
    autoMapSocket.current.send("Auto_Map");
  };

  const handleStopAutoMapping = () => {
    if (!autoMapSocket.current || autoMapSocket.current.readyState !== WebSocket.OPEN) {
      console.warn("⚠️ Cannot stop auto mapping: WebSocket not connected.");
      return;
    }

    console.log("📡 Sending stop_auto_map command...");
    autoMapSocket.current.send("stop_auto_map");
  };

  const MainContent = (
    <>
      <View style={{ flex: 1 }}>
        <HeaderBar title="Mapping Station" ws={ws.current} onRefresh={() => {}} />
        <View style={styles.container}>
          <View style={styles.topRow}>
            <View style={styles.modeSection}>
              <Text style={styles.label}>Mode</Text>
              <TouchableOpacity style={styles.dropdownButton} onPress={() => setShowModeDropdown(true)}>
                <Text style={styles.dropdownText}>{mappingMode === 'manual' ? 'Manual' : 'Auto'} ▼</Text>
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
            <TouchableOpacity style={styles.modalOverlay} onPress={() => setShowModeDropdown(false)}>
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
                    <Text style={styles.modalItemText}>{mode === 'manual' ? 'Manual' : 'Auto'}</Text>
                  </TouchableOpacity>
                ))}
              </View>
            </TouchableOpacity>
          </Modal>

          <View style={styles.mapViewBox}><MapViewBox slamRunning={slamRunning} /></View>

          {mappingMode === 'manual' ? (
            <>
              <View style={styles.rowBetween}>
                <View style={[styles.flexItem, { marginRight: 10 }]}>
                  <Text style={styles.label}>Control Type</Text>
                  <TouchableOpacity style={[styles.dropdownButton, { width: '100%' }]} onPress={() => setShowControlDropdown(true)}>
                    <Text style={styles.dropdownText}>{controlMode === 'buttons' ? 'Button Control' : 'Joystick Control'} ▼</Text>
                  </TouchableOpacity>
                </View>
                <View style={styles.flexItem}>
                  <Text style={styles.label}>SLAM</Text>
                  <TouchableOpacity
                    style={[styles.slamToggleButton, { backgroundColor: slamRunning ? '#00cc66' : '#444', width: '100%' }]}
                    onPress={toggleSlam}
                  >
                    <Text style={styles.slamToggleText}>{slamRunning ? 'Stop SLAM' : 'Start SLAM'}</Text>
                  </TouchableOpacity>
                </View>
              </View>

              <Modal visible={showControlDropdown} transparent animationType="fade">
                <TouchableOpacity style={styles.modalOverlay} onPress={() => setShowControlDropdown(false)}>
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
                        <Text style={styles.modalItemText}>{mode === 'buttons' ? 'Button Control' : 'Joystick Control'}</Text>
                      </TouchableOpacity>
                    ))}
                  </View>
                </TouchableOpacity>
              </Modal>

              <View style={styles.manualControlArea}>
                {controlMode === 'buttons' ? (
                  <ButtonControl ws={ws} linearSpeed={0.5} angularSpeed={1.0} enabled={true} />
                ) : (
                  <View>
                    <JoystickControl ws={ws} linearSpeed={0.5} angularSpeed={1.0} enabled={true} />
                  </View>
                )}
              </View>
            </>
          ) : (
            <View style={styles.buttonRow}>
              <TouchableOpacity style={styles.controlButton} onPress={handleAutoMapping}>
                <Text style={styles.controlButtonText}>Start</Text>
              </TouchableOpacity>
              <TouchableOpacity style={styles.controlButton} onPress={handleStopAutoMapping}>
                <Text style={styles.controlButtonText}>Stop</Text>
              </TouchableOpacity>
            </View>
          )}

          <TouchableOpacity style={styles.saveButton} onPress={handleSaveMap}>
            <Text style={styles.saveButtonText}>Save Map</Text>
          </TouchableOpacity>

          {savedStatus && <Text style={styles.savedStatus}>{savedStatus}</Text>}
        </View>
      </View>
    </>
  );

  return keyboardVisible ? (
    <SafeAreaView style={styles.safeArea}>
      <TouchableWithoutFeedback onPress={Keyboard.dismiss} accessible={false}>
        {MainContent}
      </TouchableWithoutFeedback>
    </SafeAreaView>
  ) : (
    <SafeAreaView style={styles.safeArea}>{MainContent}</SafeAreaView>
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
    flex:1,
    marginRight: 10,
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
    height: 300, 
    justifyContent: 'center',
    alignItems: 'center',
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
  rowBetween: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'flex-start',
    width: '100%',
    marginTop: 20,
  },
  slamToggleWrapper: {
    flex:1,
    alignItems: 'flex-end',
  },
  slamToggleButton: {
    backgroundColor: '#333',
    paddingVertical: 10,
    paddingHorizontal: 12,
    borderRadius: 10,
  },
  slamToggleText: {
    color: '#fff',
    fontWeight: '600',
  },
  flexItem: {
    justifyContent: 'space-between',
    
  },
});