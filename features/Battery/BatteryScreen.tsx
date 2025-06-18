import React, { useEffect, useRef, useState } from 'react';
import {
  View,
  Text,
  StyleSheet,
  SafeAreaView,
  ActivityIndicator,
  ScrollView,
  Dimensions,
  LayoutAnimation,
  Platform,
  UIManager,
} from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { AnimatedCircularProgress } from 'react-native-circular-progress';
import { LineChart } from 'react-native-chart-kit';
import HeaderBar from '@/components/HeaderBar';

if (Platform.OS === 'android' && UIManager.setLayoutAnimationEnabledExperimental) {
  UIManager.setLayoutAnimationEnabledExperimental(true);
}

export default function BatteryScreen() {
  const [robotIp, setRobotIp] = useState<string | null>(null);
  const [battery, setBattery] = useState<number | null>(null);
  const [voltage, setVoltage] = useState<number | null>(null);
  const [current, setCurrent] = useState<number | null>(null);
  const [temperature, setTemperature] = useState<number | null>(null);
  const [chargingStatus, setChargingStatus] = useState<string>('Unavailable');
  const [batteryHistory, setBatteryHistory] = useState<number[]>([]);
  const lastRecorded = useRef<number | null>(null);
  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    const fetchIpAndConnect = async () => {
      const ip = await AsyncStorage.getItem('robotIp');
      if (ip) {
        setRobotIp(ip);
      } else {
        console.warn('No IP address stored. Please log in again.');
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

  const connectWebSocket = (ip: string) => {
    const topicPort = 9094; // specific to /battery_state via multiplexer
    ws.current = new WebSocket(`ws://${ip}:${topicPort}`);

    ws.current.onopen = () => {
      console.log('✅ Connected to multiplexer /battery_state');
    };

    ws.current.onmessage = (e) => {
      const data = JSON.parse(e.data);
      if (!data) return;

      LayoutAnimation.configureNext(LayoutAnimation.Presets.easeInEaseOut);

      if (data.percentage !== undefined) {
        const percentage = data.percentage * 100;
        setBattery(percentage);

        const rounded = Math.round(percentage);
        if (
          lastRecorded.current === null ||
          Math.abs(rounded - lastRecorded.current) >= 1
        ) {
          setBatteryHistory((prev) => [...prev.slice(-29), rounded]);
          lastRecorded.current = rounded;
        }
      }

      if (data.voltage !== undefined) setVoltage(data.voltage);
      if (data.current !== undefined) setCurrent(data.current);
      if (data.temperature !== undefined) setTemperature(data.temperature);

      if (data.current !== undefined) {
        if (data.current > 0) setChargingStatus('Charging ⚡️');
        else if (data.current < 0) setChargingStatus('Discharging 🔋');
        else setChargingStatus('Idle');
      } else {
        setChargingStatus('Unavailable');
      }
    };

    ws.current.onerror = (err) => {
      console.error('WebSocket error:', err.message);
    };

    ws.current.onclose = () => {
      console.warn('WebSocket closed');
    };
  };

  const getGaugeColor = (level: number | null) => {
    if (level === null) return '#555';
    if (level >= 60) return '#00ff88';
    if (level >= 30) return '#ffaa00';
    return '#ff4444';
  };

  return (
    <SafeAreaView style={styles.container}>
      <HeaderBar
        title="TurtleBot Battery"
        ws={ws.current}
        onRefresh={() => {
          ws.current?.close();
          if (robotIp) connectWebSocket(robotIp);
        }}
      />

      {battery === null ? (
        <ActivityIndicator size="large" color="#00ffcc" />
      ) : (
        <ScrollView contentContainerStyle={styles.content}>
          {battery < 20 && (
            <View style={styles.warningBox}>
              <Text style={styles.warningText}>⚠️ Battery critically low!</Text>
            </View>
          )}

          <Text style={styles.gaugeLabel}>Battery Level</Text>
          <AnimatedCircularProgress
            size={200}
            width={16}
            fill={battery}
            tintColor={getGaugeColor(battery)}
            backgroundColor="#333"
            rotation={0}
            lineCap="round"
            style={styles.gaugeShadow}
          >
            {() => (
              <Text style={styles.batteryText}>{battery.toFixed(1)}%</Text>
            )}
          </AnimatedCircularProgress>

          <View style={styles.infoCard}>
            <InfoItem label="Status" value={chargingStatus} />
            <InfoItem
              label="Voltage"
              value={voltage !== null ? `${voltage.toFixed(2)} V` : '–'}
            />
            <InfoItem
              label="Current"
              value={current !== null ? `${current.toFixed(2)} A` : '–'}
            />
            <InfoItem
              label="Temperature"
              value={temperature !== null ? `${temperature.toFixed(1)} °C` : '–'}
            />
          </View>

          <Text style={styles.historyTitle}>Battery History</Text>
          <LineChart
            data={{
              labels: Array(batteryHistory.length).fill(''),
              datasets: [
                {
                  data: batteryHistory,
                },
              ],
            }}
            width={Dimensions.get('window').width - 40}
            height={160}
            chartConfig={{
              backgroundColor: '#1e1e1e',
              backgroundGradientFrom: '#1e1e1e',
              backgroundGradientTo: '#1e1e1e',
              color: () => '#00ffcc',
              labelColor: () => '#ccc',
              propsForDots: {
                r: '2',
                strokeWidth: '1',
                stroke: '#00ffcc',
              },
              propsForBackgroundLines: {
                stroke: '#333',
              },
            }}
            bezier
            style={styles.chart}
          />
        </ScrollView>
      )}
    </SafeAreaView>
  );
}

const InfoItem = ({ label, value }: { label: string; value: string }) => (
  <View style={styles.infoRow}>
    <Text style={styles.label}>{label}:</Text>
    <Text style={styles.value}>{value}</Text>
  </View>
);

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#1e1e1e',
    marginTop: -20,
  },
  content: {
    alignItems: 'center',
    paddingVertical: 30,
  },
  title: {
    fontSize: 22,
    color: '#00ffcc',
  },
  gaugeLabel: {
    fontSize: 20,
    color: '#ccc',
    marginTop: 10,
    marginBottom: 30,
  },
  gaugeShadow: {
    shadowColor: '#00ffcc',
    shadowOffset: { width: 0, height: 0 },
    shadowOpacity: 0.4,
    shadowRadius: 10,
  },
  batteryText: {
    fontSize: 30,
    fontWeight: 'bold',
    color: '#fff',
    marginTop: 15,
  },
  warningBox: {
    backgroundColor: '#ff4444',
    padding: 10,
    borderRadius: 10,
    marginBottom: 20,
  },
  warningText: {
    color: 'white',
    fontWeight: 'bold',
  },
  infoCard: {
    marginTop: 30,
    width: '85%',
    backgroundColor: '#2a2a2a',
    padding: 16,
    borderRadius: 12,
    shadowColor: '#000',
    shadowOffset: { width: 0, height: 2 },
    shadowOpacity: 0.3,
    shadowRadius: 4,
    elevation: 5,
  },
  infoRow: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    marginVertical: 8,
  },
  label: {
    fontSize: 16,
    color: '#aaa',
  },
  value: {
    fontSize: 16,
    color: '#fff',
  },
  chart: {
    marginTop: 20,
    borderRadius: 10,
  },
  historyTitle: {
    fontSize: 20,
    fontWeight: '600',
    color: '#00ffcc',
    marginTop: 40,
  },
});