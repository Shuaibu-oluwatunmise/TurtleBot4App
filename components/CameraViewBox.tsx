import React, { useEffect, useRef } from 'react';
import { Image, StyleSheet, View } from 'react-native';

export default function CameraViewBox({ robotIp }: { robotIp: string | null }) {
  const ws = useRef<WebSocket | null>(null);
  const [imageData, setImageData] = React.useState<string | null>(null);

  useEffect(() => {
    if (!robotIp) return;
    ws.current = new WebSocket(`ws://${robotIp}:9100`); 

    ws.current.onmessage = (msg) => {
      const { type, image } = JSON.parse(msg.data);
      if (type === 'camera') {
        setImageData(`data:image/jpeg;base64,${image}`);
      }
    };

    ws.current.onerror = (err) => {
      console.error("❌ Camera WebSocket error:", err.message);
    };

    return () => ws.current?.close();
  }, [robotIp]);

  return (
    <View style={styles.container}>
      {imageData ? (
        <Image source={{ uri: imageData }} style={styles.image} resizeMode="cover" />
      ) : (
        <View style={styles.placeholder} />
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    borderRadius: 12,
    overflow: 'hidden',
  },
  image: {
    width: '100%',
    height: '100%',
  },
  placeholder: {
    backgroundColor: '#444',
    width: '100%',
    height: '100%',
  },
});
