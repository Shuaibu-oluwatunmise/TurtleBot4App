import React, { useEffect, useState } from 'react';
import { View, Text, StyleSheet, Image, Pressable } from 'react-native';
import RefreshWebSocketButton from './RefreshWebSocketButton';
import { useNavigation } from '@react-navigation/native';
import { NativeStackNavigationProp } from '@react-navigation/native-stack';
import { RootStackParamList } from '@/App'; // adjust path if needed

interface HeaderProps {
  title: string;
  ws: WebSocket | null;
  onRefresh: () => void;
}

type NavigationProp = NativeStackNavigationProp<RootStackParamList, 'Menu'>;

export default function HeaderBar({ title, ws, onRefresh }: HeaderProps) {
  const [connected, setConnected] = useState(false);
  const navigation = useNavigation<NavigationProp>();

  useEffect(() => {
    if (!ws) return;

    const checkConnection = () => {
      setConnected(ws.readyState === WebSocket.OPEN);
    };

    checkConnection();
    const interval = setInterval(checkConnection, 1000);
    return () => clearInterval(interval);
  }, [ws]);

  return (
    <View style={styles.header}>
      <View style={styles.leftRow}>
        <Pressable onPress={() => navigation.navigate('Menu')}>
          <Image
            source={require('@/assets/images/turtleLogo.png')}
            style={styles.icon}
          />
        </Pressable>
        <Text style={styles.title}>{title}</Text>
      </View>
      <View style={styles.statusRow}>
        <View
          style={[
            styles.statusLight,
            { backgroundColor: connected ? '#00ff00' : '#ff4444' },
          ]}
        />
        <RefreshWebSocketButton onPress={onRefresh} />
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  header: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    alignItems: 'center',
    paddingHorizontal: 20,
    paddingTop: 10,
    paddingBottom: 8,
    borderBottomWidth: 1,
    borderBottomColor: '#2a2a2a',
    backgroundColor: '#1e1e1e',
  },
  leftRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 10,
  },
  title: {
    fontSize: 22,
    color: '#00ffcc',
  },
  icon: {
    width: 28,
    height: 28,
    resizeMode: 'contain',
  },
  statusRow: {
    flexDirection: 'row',
    alignItems: 'center',
    gap: 12,
  },
  statusLight: {
    width: 14,
    height: 14,
    borderRadius: 7,
  },
});
