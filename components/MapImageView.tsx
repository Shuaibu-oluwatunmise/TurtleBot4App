import React, { useEffect, useState } from 'react';
import { View, Image, StyleSheet, Text } from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';

export default function MapImageView() {
  const [imageData, setImageData] = useState<string | null>(null);

  useEffect(() => {
    let ws: WebSocket;

    const connect = async () => {
      const ip = await AsyncStorage.getItem('robotIp');
      if (!ip) return;

      ws = new WebSocket(`ws://${ip}:9096`);
      ws.onmessage = (event) => setImageData(event.data);
      ws.onerror = (e) => console.error('❌ Map image error:', e.message);
      ws.onclose = () => console.log('🔌 Map image socket closed');
    };

    connect();
    return () => ws?.close();
  }, []);

  return (
    <View style={styles.wrapper}>
      {imageData ? (
        <Image
          source={{ uri: `data:image/png;base64,${imageData}` }}
          style={styles.image}
          resizeMode="contain"
        />
      ) : (
        <Text style={styles.placeholder}>Waiting for map...</Text>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  wrapper: {
    width: '100%',
    height: 200,
    backgroundColor: '#2a2a2a',
    borderRadius: 12,
    borderWidth: 1,
    borderColor: '#00ffcc',
    overflow: 'hidden',
    alignItems: 'center',
    justifyContent: 'center',
  },
  image: {
    width: '100%',
    height: '100%',
  },
  placeholder: {
    color: '#999',
  },
});
