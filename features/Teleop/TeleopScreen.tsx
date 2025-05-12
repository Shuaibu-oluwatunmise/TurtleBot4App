import React, { useRef, useState, useEffect } from 'react';
import { View, Text, StyleSheet, Pressable, Modal, TouchableOpacity, SafeAreaView, ScrollView } from 'react-native';
import Slider from '@react-native-community/slider';
import AsyncStorage from '@react-native-async-storage/async-storage';
import JoystickControl from '@/components/JoystickControl';
import LiveTelemetry from '@/components/LiveTelemetry';
import HeaderBar from '@/components/HeaderBar';
import WheelStatus from '@/components/WheelStatus';
import ButtonControl from '@/components/ButtonControl'; 


const DROPDOWN_OPTIONS = {
  mode: [
    { label: 'Button Control', value: 'buttons' },
    { label: 'Joystick Control', value: 'joystick' },
  ],
  speed: [
    { label: 'Slow', value: '0.2-0.5' },
    { label: 'Normal', value: '0.5-1.0' },
    { label: 'Fast', value: '1.0-2.0' },
  ],
};

export default function TeleopScreen() {
  const ws = useRef<WebSocket | null>(null);
  const [robotIp, setRobotIp] = useState<string | null>(null);
  const [mode, setMode] = useState<'buttons' | 'joystick'>('buttons');
  const [linearSpeed, setLinearSpeed] = useState(0.5);
  const [angularSpeed, setAngularSpeed] = useState(1.0);
  const [enabled, setEnabled] = useState(true);
  const [showModeDropdown, setShowModeDropdown] = useState(false);
  const [showSpeedDropdown, setShowSpeedDropdown] = useState(false);

  const connectWebSocket = (ip: string) => {
    ws.current = new WebSocket(`ws://${ip}:9090`);
  };

  const sendCommand = (linear: number, angular: number) => {
    if (!enabled || !ws.current || ws.current.readyState !== WebSocket.OPEN) return;
    ws.current.send(
      JSON.stringify({
        op: 'publish',
        topic: '/cmd_vel',
        msg: {
          header: { stamp: { sec: 0, nanosec: 0 }, frame_id: '' },
          twist: {
            linear: { x: linear, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: angular },
          },
        },
      })
    );
  };

  useEffect(() => {
    const fetchIpAndConnect = async () => {
      const ip = await AsyncStorage.getItem('robotIp');
      if (ip) {
        setRobotIp(ip);
      } else {
        console.warn('No IP address found in storage.');
      }
    };
    fetchIpAndConnect();
  }, []);

  useEffect(() => {
    if (robotIp) {
      connectWebSocket(robotIp);
      return () => ws.current?.close();
    }
  }, [robotIp]);

  const moveInterval = useRef<NodeJS.Timeout | null>(null);

  const handlePressIn = (linear: number, angular: number) => {
    moveInterval.current = setInterval(() => sendCommand(linear, angular), 100);
  };

  const handlePressOut = () => {
    if (moveInterval.current) clearInterval(moveInterval.current);
    sendCommand(0, 0);
  };

  const applyPreset = (linear: number, angular: number) => {
    setLinearSpeed(linear);
    setAngularSpeed(angular);
  };

  return (
    <SafeAreaView style={styles.safeArea}>
      <HeaderBar
        title="Teleoperation"
        ws={ws.current}
        onRefresh={() => {
          ws.current?.close();
          if (robotIp) connectWebSocket(robotIp);
        }}
      />
        <View style={styles.scrollContainer}>
            <View style={styles.InfoArea}>
                <Text style={styles.sectionTitle}>Live Info</Text>
                <View style={styles.infoSection}>
                <LiveTelemetry />
                <WheelStatus />
                </View>
            </View>

            <View style={styles.EntireControlArea}>
                <Text style={styles.sectionTitle}>Control Panel</Text>
                <View style={styles.controlBar}>
                <View style={styles.dropdownContainer}>
                    <Text style={styles.label}>Mode</Text>
                    <TouchableOpacity onPress={() => setShowModeDropdown(true)} style={styles.dropdownButton}>
                    <Text style={styles.dropdownText}>{mode === 'buttons' ? 'Button Control' : 'Joystick Control'} ▼</Text>
                    </TouchableOpacity>
                </View>

                <Pressable
                    style={[styles.toggleButton, { backgroundColor: enabled ? '#505050' : '#040404' }]}
                    onPress={() => setEnabled(!enabled)}
                >
                    <Text style={styles.toggleText}>{enabled ? 'Disable' : 'Enable'}</Text>
                </Pressable>

                <View style={styles.dropdownContainer}>
                    <Text style={styles.label}>Preset Speed</Text>
                    <TouchableOpacity onPress={() => setShowSpeedDropdown(true)} style={styles.dropdownButton}>
                    <Text style={styles.dropdownText}>
                        {linearSpeed === 0.2 ? 'Slow' : linearSpeed === 0.5 ? 'Normal' : 'Fast'} ▼
                    </Text>
                    </TouchableOpacity>
                </View>
                </View>

                <Modal visible={showModeDropdown} transparent animationType="fade">
                <TouchableOpacity style={styles.modalOverlay} onPress={() => setShowModeDropdown(false)}>
                    <View style={styles.modalContent}>
                    {DROPDOWN_OPTIONS.mode.map((option) => (
                        <Pressable
                        key={option.value}
                        style={styles.modalItem}
                        onPress={() => {
                            setMode(option.value as 'buttons' | 'joystick');
                            setShowModeDropdown(false);
                        }}
                        >
                        <Text style={styles.modalItemText}>{option.label}</Text>
                        </Pressable>
                    ))}
                    </View>
                </TouchableOpacity>
                </Modal>

                <Modal visible={showSpeedDropdown} transparent animationType="fade">
                <TouchableOpacity style={styles.modalOverlay} onPress={() => setShowSpeedDropdown(false)}>
                    <View style={styles.modalContent}>
                    {DROPDOWN_OPTIONS.speed.map((option) => (
                        <Pressable
                        key={option.value}
                        style={styles.modalItem}
                        onPress={() => {
                            const [lin, ang] = option.value.split('-').map(Number);
                            applyPreset(lin, ang);
                            setShowSpeedDropdown(false);
                        }}
                        >
                        <Text style={styles.modalItemText}>{option.label}</Text>
                        </Pressable>
                    ))}
                    </View>
                </TouchableOpacity>
                </Modal>

                <View style={styles.controlSection}>
                  <Text style={styles.modeText}>{mode === 'buttons' ? 'Button Control' : 'Joystick Control'}</Text>
                  <View style={styles.controlArea}>
                      {mode === 'buttons' ? (
                        <ButtonControl
                          linearSpeed={linearSpeed}
                          angularSpeed={angularSpeed}
                          sendCommand={sendCommand}
                          enabled={enabled}
                        />
                      ) : (
                      <JoystickControl 
                        ws={ws} linearSpeed={linearSpeed} 
                        angularSpeed={angularSpeed} 
                        enabled={enabled} 
                      />
                      )}
                  </View>
                </View>

                <View style={styles.sliderContainer}>
                <Text style={styles.sliderLabel}>Linear Speed: {linearSpeed.toFixed(2)} m/s</Text>
                <Slider
                    style={styles.slider}
                    minimumValue={0.1}
                    maximumValue={1.0}
                    value={linearSpeed}
                    onValueChange={setLinearSpeed}
                    minimumTrackTintColor="#00ffcc"
                    maximumTrackTintColor="#555"
                />

                <Text style={styles.sliderLabel}>Angular Speed: {angularSpeed.toFixed(2)} rad/s</Text>
                <Slider
                    style={styles.slider}
                    minimumValue={0.1}
                    maximumValue={2.0}
                    value={angularSpeed}
                    onValueChange={setAngularSpeed}
                    minimumTrackTintColor="#00ffcc"
                    maximumTrackTintColor="#555"
                />
                </View>
            </View>
      </View>
    </SafeAreaView >
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#1e1e1e',
    paddingHorizontal: 16,
    paddingTop: 20,
  },
  safeArea: {
    flex: 1,
    backgroundColor: '#1e1e1e',
    marginTop: -20
  },
  scrollContainer: {
    paddingHorizontal: 16,
    paddingTop: 20,
    paddingBottom: 40,
  },  
  sectionTitle: {
    color: '#00ffcc',
    fontSize: 16,
    fontWeight: 'bold',
    textAlign: 'center',
    marginBottom: 8,
    marginTop: 8,
  },
  infoSection: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    marginBottom: 12,
  },
  controlBar: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    marginBottom: 10,
  },
  dropdownContainer: {
    flex: 1,
    marginHorizontal: 4,
  },
  label: {
    color: '#fff',
    marginBottom: 4,
    marginLeft: 16,
  },
  dropdownButton: {
    backgroundColor: '#040404',
    padding: 10,
    borderRadius: 10,
    alignItems: 'center',
  },
  dropdownText: {
    color: '#00ffcc',
  },
  controlSection: {
    alignItems: 'center',
    marginBottom: 10,
  },
  controlArea: {
    width: '100%',
    minHeight: 300,
    backgroundColor: '#040404',
    borderRadius: 16,
    padding: 12,
    justifyContent: 'center',
  },
  InfoArea: {
    backgroundColor: '#2a2a2a',
    borderRadius: 16,
    paddingHorizontal: 12,
    justifyContent: 'center',
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.3,
    shadowRadius: 4,
    elevation: 5,
  },
  EntireControlArea: {
    marginTop: 20,
    backgroundColor: '#2a2a2a',
    borderRadius: 16,
    paddingHorizontal: 12,
    justifyContent: 'center',
    paddingTop: 10,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.3,
    shadowRadius: 4,
    elevation: 5,
  },
  buttonRow: {
    flexDirection: 'row',
    justifyContent: 'space-evenly',
    width: '100%',
  },  
  modeText: {
    fontSize: 16,
    color: '#fff',
    marginBottom: 6,
  },
  controlPad: {
    justifyContent: 'center',
    alignItems: 'center',
  },
  arrowButton: {
    width: 70,
    height: 70,
    backgroundColor: '#333',
    borderRadius: 16,
    justifyContent: 'center',
    alignItems: 'center',
    margin: 10,
  },
  arrowText: {
    fontSize: 30,
    color: '#fff',
  },
  middleRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
  },
  toggleButton: {
    paddingVertical: 10,
    paddingHorizontal: 10,
    borderRadius: 10,
    marginHorizontal: 10,
    marginBottom: -20
  },
  toggleText: {
    color: '#00ffcc',
    fontWeight: 'bold',
  },
  sliderContainer: {
    marginBottom: 30,
  },
  sliderLabel: {
    color: '#fff',
    marginTop: 10,
  },
  slider: {
    width: '100%',
    height: 20,
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
  controllabeltext:{
    color: '#fff',
    fontSize: 16,
    textAlign: 'center',
  }
});