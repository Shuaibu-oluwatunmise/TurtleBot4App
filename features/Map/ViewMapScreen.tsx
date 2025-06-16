import React, { useEffect, useState, useRef } from 'react';
import {
  View,
  Text,
  FlatList,
  Image,
  StyleSheet,
  SafeAreaView,
  ActivityIndicator,
  Dimensions,
  TouchableOpacity,
  Pressable,
  Alert,
  Vibration,
} from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { useNavigation } from '@react-navigation/native';
import HeaderBar from '@/components/HeaderBar';

const NUM_COLUMNS = 2;
const CARD_MARGIN = 10;
const SCREEN_PADDING = 20;

export default function MapGalleryScreen() {
  const [maps, setMaps] = useState([]);
  const [loading, setLoading] = useState(true);
  const [robotIp, setRobotIp] = useState(null);
  const ws = useRef(null);
  const [selected, setSelected] = useState(new Set());
  const [selectMode, setSelectMode] = useState(false);
  const navigation = useNavigation();

  useEffect(() => {
    const fetchIpAndConnect = async () => {
      const ip = await AsyncStorage.getItem('robotIp');
      if (!ip) return;
      setRobotIp(ip);

      try {
        ws.current = new WebSocket(`ws://${ip}:9097`);
        ws.current.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data);
            if (Array.isArray(data)) {
              setMaps(data);
              setLoading(false);
            }
          } catch (e) {
            console.error('❌ Failed to parse saved maps:', e);
          }
        };
      } catch (error) {
        console.error('❌ WebSocket connection failed:', error);
      }
    };

    fetchIpAndConnect();
    return () => ws.current?.close();
  }, []);

  const screenWidth = Dimensions.get('window').width;
  const cardWidth = (screenWidth - SCREEN_PADDING * 2 - CARD_MARGIN * (NUM_COLUMNS - 1)) / NUM_COLUMNS;

  const handleLongPress = () => {
    Vibration.vibrate(50);
    setSelectMode(true);
  };

  const toggleSelect = (filename) => {
    const newSelected = new Set(selected);
    if (newSelected.has(filename)) newSelected.delete(filename);
    else newSelected.add(filename);
    setSelected(newSelected);
  };

  const handleDelete = () => {
    Alert.alert(
      'Confirm Deletion',
      `Are you sure you want to delete ${selected.size} map(s)?`,
      [
        { text: 'Cancel', style: 'cancel' },
        {
          text: 'Yes',
          onPress: () => {
            const remaining = maps.filter((m) => !selected.has(m.filename));
            setMaps(remaining);
            setSelectMode(false);
            setSelected(new Set());
          },
        },
      ]
    );
  };

  const renderMap = ({ item }) => (
    <Pressable
      onLongPress={handleLongPress}
      onPress={() =>
        selectMode
          ? toggleSelect(item.filename)
          : navigation.navigate('FullGallery', {
              maps,
              initialIndex: maps.findIndex((m) => m.filename === item.filename),
            })
      }
      style={[styles.card, { width: cardWidth }]}
    >
      <Image
        source={{ uri: `data:image/png;base64,${item.base64}` }}
        style={styles.image}
        resizeMode="cover"
      />
      {selectMode && (
        <View style={styles.circleWrapper}>
          <View style={[styles.circle, selected.has(item.filename) && styles.circleFilled]} />
        </View>
      )}
      <Text style={styles.filename}>{item.filename}</Text>
    </Pressable>
  );

  return (
    <SafeAreaView style={styles.safeArea}>
      <HeaderBar title="View Maps" ws={ws.current} onRefresh={() => {}} />
      <View style={styles.container}>
        {loading ? (
          <ActivityIndicator color="#00ffcc" size="large" />
        ) : (
          <FlatList
            data={maps}
            keyExtractor={(item) => item.filename}
            renderItem={renderMap}
            numColumns={NUM_COLUMNS}
            contentContainerStyle={styles.list}
            columnWrapperStyle={styles.rowWrapper}
          />
        )}

        {selectMode && (
          <TouchableOpacity
            style={[styles.deleteButton, selected.size === 0 && styles.deleteDisabled]}
            disabled={selected.size === 0}
            onPress={handleDelete}
          >
            <Text style={styles.deleteText}>Delete</Text>
          </TouchableOpacity>
        )}
      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#121212',
  },
  safeArea: {
    flex: 1,
    backgroundColor: '#1e1e1e',
  },
  title: {
    color: '#00ffcc',
    fontSize: 24,
    fontWeight: 'bold',
    marginTop: 20,
    marginBottom: 10,
    alignSelf: 'center',
  },
  list: {
    paddingBottom: 40,
  },
  rowWrapper: {
    justifyContent: 'space-between',
    marginBottom: 15,
    paddingHorizontal: 20,
    paddingTop: 20,
  },
  card: {
    backgroundColor: '#1e1e1e',
    borderRadius: 12,
    padding: 8,
    alignItems: 'center',
    borderColor: '#00ffcc',
    borderWidth: 1,
  },
  image: {
    width: '100%',
    height: 160,
    borderRadius: 8,
    marginBottom: 8,
  },
  filename: {
    color: '#fff',
    fontSize: 12,
    textAlign: 'center',
  },
  circleWrapper: {
    position: 'absolute',
    top: 8,
    right: 8,
    width: 24,
    height: 24,
    borderRadius: 12,
    borderWidth: 2,
    borderColor: '#00ffcc',
    backgroundColor: '#1e1e1e',
    justifyContent: 'center',
    alignItems: 'center',
  },
  circle: {
    width: 12,
    height: 12,
    borderRadius: 6,
    backgroundColor: 'transparent',
  },
  circleFilled: {
    backgroundColor: '#00ffcc',
  },
  deleteButton: {
    backgroundColor: '#ff0033',
    padding: 16,
    borderRadius: 12,
    position: 'absolute',
    bottom: 20,
    alignSelf: 'center',
    paddingHorizontal: 40,
  },
  deleteDisabled: {
    backgroundColor: '#555',
  },
  deleteText: {
    color: '#fff',
    fontWeight: 'bold',
    fontSize: 16,
  },
});