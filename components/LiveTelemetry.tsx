import React, { useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';

export default function LiveTelemetry() {
  const [position, setPosition] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [yaw, setYaw] = useState<number>(0);
  const [direction, setDirection] = useState<string>('–');
  const [robotIp, setRobotIp] = useState<string | null>(null);
  const ws = useRef<WebSocket | null>(null);

  useEffect(() => {
    const fetchIp = async () => {
      const ip = await AsyncStorage.getItem('robotIp');
      if (ip) setRobotIp(ip);
    };
    fetchIp();
  }, []);

  useEffect(() => {
    if (!robotIp) return;

    ws.current = new WebSocket(`ws://${robotIp}:9090`);

    ws.current.onopen = () => {
      ws.current?.send(
        JSON.stringify({
          op: 'subscribe',
          topic: '/odom',
        })
      );
    };

    ws.current.onmessage = (e) => {
      const msg = JSON.parse(e.data);
      const data = msg.msg;
      if (!data) return;

      const pos = data.pose?.pose?.position;
      const ori = data.pose?.pose?.orientation;

      if (pos && ori) {
        setPosition({ x: pos.x, y: pos.y });

        // Calculate yaw from quaternion
        const { x, y, z, w } = ori;
        const siny_cosp = 2 * (w * z + x * y);
        const cosy_cosp = 1 - 2 * (y * y + z * z);
        const yawAngle = Math.atan2(siny_cosp, cosy_cosp);
        const yawDeg = (yawAngle * 180) / Math.PI;
        setYaw(yawDeg);

        const dirs = ['North', 'North-East', 'East', 'South-East', 'South', 'South-West', 'West', 'NorthWest'];
        const index = Math.round((yawDeg % 360) / 45) % 8;
        setDirection(dirs[(index + 8) % 8]);
      }
    };

    ws.current.onerror = (err) => {
      console.error('Telemetry WebSocket error:', err.message);
    };

    return () => ws.current?.close();
  }, [robotIp]);

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