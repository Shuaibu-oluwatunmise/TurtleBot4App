import React, { useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';

export default function WheelStatus() {
  const [leftPos, setLeftPos] = useState<number | null>(null);
  const [rightPos, setRightPos] = useState<number | null>(null);
  const [leftVel, setLeftVel] = useState<number | null>(null);
  const [rightVel, setRightVel] = useState<number | null>(null);
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
          topic: '/joint_states',
        })
      );
    };

    ws.current.onmessage = (e) => {
      const msg = JSON.parse(e.data);
      const data = msg.msg;
      if (!data) return;

      // Assumes index 1 = left wheel, index 3 = right wheel
      if (data.position?.length >= 4) {
        setLeftPos(data.position[1]);
        setRightPos(data.position[3]);
      }
      if (data.velocity?.length >= 4) {
        setLeftVel(data.velocity[1]);
        setRightVel(data.velocity[3]);
      } else {
        setLeftVel(null);
        setRightVel(null);
      }
    };

    return () => ws.current?.close();
  }, [robotIp]);

  return (
    <View style={styles.container}>
      <Text style={styles.header}>🛞 Wheel Status</Text>
      <Text style={styles.item}>Left Position  : {leftPos !== null ? leftPos.toFixed(2) : 'Unavailable'}</Text>
      <Text style={styles.item}>Right Position : {rightPos !== null ? rightPos.toFixed(2) : 'Unavailable'}</Text>
      <Text style={styles.item}>Left Velocity  : {leftVel !== null ? `${leftVel.toFixed(2)} rad/s` : 'Unavailable'}</Text>
      <Text style={styles.item}>Right Velocity : {rightVel !== null ? `${rightVel.toFixed(2)} rad/s` : 'Unavailable'}</Text>
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
