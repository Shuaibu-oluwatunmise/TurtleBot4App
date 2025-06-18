import React, { useEffect, useRef, useState } from 'react';
import { View, Image, Text, StyleSheet } from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';

export default function MapViewBox({ slamRunning }: { slamRunning: boolean }) {
  const [robotIp, setRobotIp] = useState<string | null>(null);
  const [imageData, setImageData] = useState<string | null>(null);
  const imageSocket = useRef<WebSocket | null>(null);

  useEffect(() => {
    const fetchIp = async () => {
      const ip = await AsyncStorage.getItem('robotIp');
      if (ip) setRobotIp(ip);
    };
    fetchIp();
  }, []);

  useEffect(() => {
    if (!robotIp || !slamRunning) {
      setImageData(null);
      return;
    }

    imageSocket.current = new WebSocket(`ws://${robotIp}:9096`);

    imageSocket.current.onmessage = (event) => {
      setImageData(event.data);
    };

    imageSocket.current.onerror = (e) => {
      console.error('🛑 Image socket error:', e.message);
    };

    return () => imageSocket.current?.close();
  }, [robotIp, slamRunning]);

  return (
    <View style={styles.wrapper}>
      {slamRunning && imageData ? (
        <Image
          source={{ uri: `data:image/png;base64,${imageData}` }}
          style={styles.mapImage}
          resizeMode="contain"
        />
      ) : (
        <View style={styles.placeholderContainer}>
          <Text style={styles.placeholderText}>🕓 Waiting for SLAM...</Text>
        </View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  wrapper: {
    flex: 1,
    backgroundColor: '#111',
    justifyContent: 'center',
    alignItems: 'center',
  },
  mapImage: {
    width: '100%',
    height: '100%',
  },
  placeholderContainer: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
  },
  placeholderText: {
    color: '#888',
    fontSize: 16,
    fontStyle: 'italic',
  },
});
