import gzip
import xml.etree.ElementTree as ET

def find_valid_links(network_path, max_links=20):
    """
    Findet gültige (nicht-PT) Link-IDs aus einem MATSim-Netzwerk.
    
    Args:
        network_path: Pfad zur network.xml.gz Datei
        max_links: Maximale Anzahl anzuzeigender Links
    """
    print(f"Lese Netzwerk: {network_path}\n")
    
    valid_links = []
    pt_links = []
    total_links = 0
    
    try:
        # Öffne komprimierte XML-Datei
        with gzip.open(network_path, 'rt', encoding='utf-8') as f:
            # Iteriere durch XML ohne alles in Speicher zu laden
            for event, elem in ET.iterparse(f, events=('end',)):
                if elem.tag == 'link':
                    total_links += 1
                    link_id = elem.get('id')
                    
                    if link_id:
                        # Prüfe ob PT-Link
                        if link_id.startswith('pt_'):
                            pt_links.append(link_id)
                        else:
                            valid_links.append(link_id)
                    
                    # Speicher freigeben
                    elem.clear()
                    
                    # Früher Abbruch wenn genug Links gefunden
                    if len(valid_links) >= max_links * 2:
                        break
        
        # Ergebnisse ausgeben
        print(f"=== ERGEBNISSE ===")
        print(f"Gesamt analysierte Links: {total_links}")
        print(f"Gültige (nicht-PT) Links: {len(valid_links)}")
        print(f"PT-Links gefunden: {len(pt_links)}")
        print()
        
        # Prüfe ob Link 25443 existiert
        if '25443' in valid_links:
            print("✓ Link 25443 ist ein GÜLTIGER Link (nicht PT)")
        elif '25443' in pt_links:
            print("✗ Link 25443 ist ein PT-Link (nicht verwendbar)")
        else:
            print("✗ Link 25443 wurde nicht gefunden")
        print()
        
        # Zeige erste gültige Links
        print(f"Erste {min(max_links, len(valid_links))} gültige Link-IDs:")
        for i, link_id in enumerate(valid_links[:max_links], 1):
            print(f"  {i:2d}. {link_id}")
        
        if pt_links:
            print(f"\nErste {min(5, len(pt_links))} PT-Links (zur Info):")
            for i, link_id in enumerate(pt_links[:5], 1):
                print(f"  {i}. {link_id}")
        
        # Empfehlung
        print("\n=== EMPFEHLUNG ===")
        if valid_links:
            print(f"Verwende einen dieser Links als start_link:")
            print(f"  Beispiel: start_link=\"{valid_links[0]}\"")
        else:
            print("WARNUNG: Keine gültigen Links gefunden!")
            
    except FileNotFoundError:
        print(f"FEHLER: Datei nicht gefunden: {network_path}")
    except Exception as e:
        print(f"FEHLER beim Lesen der Datei: {e}")

if __name__ == "__main__":
    network_path = r"C:\Users\andre\Desktop\04025_MATSim\fleetpy_x_ridesync_x_bavaria\eqasim-java\output-bavaria-pipeline\bavaria_network.xml.gz"
    
    find_valid_links(network_path, max_links=20)