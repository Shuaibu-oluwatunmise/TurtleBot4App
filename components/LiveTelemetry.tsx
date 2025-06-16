import React, { useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';

export default function LiveTelemetry() {
  const [position, setPosition] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [yaw, setYaw] = useState<number>(0);
  const [direction, setDirection] = useState<string>('–');
  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    const connectWebSocket = async () => {
      const ip = await AsyncStorage.getItem('robotIp');
      if (!ip) return;

      ws.current = new WebSocket(`ws://${ip}:9093`);

      ws.current.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data);
          const pos = data.pose?.pose?.position;
          const ori = data.pose?.pose?.orientation;

          if (pos && ori) {
            setPosition({ x: pos.x, y: pos.y });

            // Convert quaternion to yaw angle
            const { x, y, z, w } = ori;
            const siny_cosp = 2 * (w * z + x * y);
            const cosy_cosp = 1 - 2 * (y * y + z * z);
            const yawAngle = Math.atan2(siny_cosp, cosy_cosp);
            const yawDeg = (yawAngle * 180) / Math.PI;
            setYaw(yawDeg);

            const dirs = ['North', 'North-East', 'East', 'South-East', 'South', 'South-West', 'West', 'North-West'];
            const index = Math.round((yawDeg % 360) / 45) % 8;
            setDirection(dirs[(index + 8) % 8]);
          }
        } catch (err) {
          console.error('Failed to parse /odom message:', err);
        }
      };

      ws.current.onerror = (err) => {
        console.error('LiveTelemetry WebSocket error:', err.message);
      };

      ws.current.onclose = () => {
        console.warn('LiveTelemetry WebSocket closed');
      };
    };

    connectWebSocket();

    return () => {
      if (ws.current) {
        ws.current.close();
        ws.current = null;
      }
    };
  }, []);

  return (
    <View style={styles.container}>
      <Text style={styles.header}>📡 Live Telemetry</Text>
      <Text style={styles.item}>x-coordinate: {position.x.toFixed(2)}</Text>
      <Text style={styles.item}>y-coordinate: {position.y.toFixed(2)}</Text>
      <Text style={styles.item}>Yaw Angle : {yaw.toFixed(1)}°</Text>
      <Text style={styles.item}>Direction : {direction}</Text>
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    paddingHorizontal: 16,
    backgroundColor: '#040404',
    borderRadius: 12,
    paddingVertical: 10,
    height: 120,
    width: 170,
  },
  header: {
    color: '#00ffcc',
    fontSize: 16,
    fontWeight: 'bold',
    marginBottom: 8,
  },
  item: {
    color: '#fff',
    fontSize: 14,
    marginBottom: 4,
  },
});