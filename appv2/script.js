// Initialize the map
const map = L.map('map').setView([40.7128, -74.0060], 4);

// Add OpenStreetMap tiles
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '© OpenStreetMap contributors',
    maxZoom: 19
}).addTo(map);

// Store markers
let markers = [];
let markerCounter = 1;

// Function to create a marker
function addMarker(lat, lng) {
    const timestamp = new Date().toISOString();
    const markerId = `MARKER-${String(markerCounter).padStart(4, '0')}`;
    
    const marker = L.marker([lat, lng]).addTo(map);
    
    // Create popup content
    const popupContent = `
        <div class="marker-popup">
            <h4>Marker Details</h4>
            <p><strong>ID:</strong> ${markerId}</p>
            <p><strong>Latitude:</strong> ${lat.toFixed(6)}</p>
            <p><strong>Longitude:</strong> ${lng.toFixed(6)}</p>
            <p><strong>Recorded:</strong> ${new Date(timestamp).toLocaleString()}</p>
        </div>
    `;
    
    marker.bindPopup(popupContent);
    
    // Store marker data
    markers.push({
        id: markerId,
        lat: lat,
        lng: lng,
        timestamp: timestamp,
        leafletMarker: marker
    });
    
    markerCounter++;
    
    // Pan to the new marker
    map.setView([lat, lng], Math.max(map.getZoom(), 10));
    
    // Save to localStorage
    saveMarkers();
}

// Save markers to localStorage
function saveMarkers() {
    const markerData = markers.map(m => ({
        id: m.id,
        lat: m.lat,
        lng: m.lng,
        timestamp: m.timestamp
    }));
    localStorage.setItem('gpsMarkers', JSON.stringify(markerData));
    localStorage.setItem('markerCounter', markerCounter);
}

// Load markers from localStorage
function loadMarkers() {
    const saved = localStorage.getItem('gpsMarkers');
    const savedCounter = localStorage.getItem('markerCounter');
    
    if (savedCounter) {
        markerCounter = parseInt(savedCounter);
    }
    
    if (saved) {
        const markerData = JSON.parse(saved);
        markerData.forEach(m => {
            const marker = L.marker([m.lat, m.lng]).addTo(map);
            
            const popupContent = `
                <div class="marker-popup">
                    <h4>Marker Details</h4>
                    <p><strong>ID:</strong> ${m.id}</p>
                    <p><strong>Latitude:</strong> ${m.lat.toFixed(6)}</p>
                    <p><strong>Longitude:</strong> ${m.lng.toFixed(6)}</p>
                    <p><strong>Recorded:</strong> ${new Date(m.timestamp).toLocaleString()}</p>
                </div>
            `;
            
            marker.bindPopup(popupContent);
            
            markers.push({
                id: m.id,
                lat: m.lat,
                lng: m.lng,
                timestamp: m.timestamp,
                leafletMarker: marker
            });
        });
        
        // Fit map to show all markers
        if (markers.length > 0) {
            const group = L.featureGroup(markers.map(m => m.leafletMarker));
            map.fitBounds(group.getBounds().pad(0.1));
        }
    }
}

// Clear all markers
function clearAllMarkers() {
    markers.forEach(m => map.removeLayer(m.leafletMarker));
    markers = [];
    markerCounter = 1;
    localStorage.removeItem('gpsMarkers');
    localStorage.removeItem('markerCounter');
}
// Load markers from JSON file in GPSData folder
async function loadMarkersFromFile(filename) {
    try {
        
        const response = await fetch(`GPSData/${filename}`);
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const markerData = await response.json();
        
        if (!Array.isArray(markerData)) {
            alert('JSON file must contain an array of markers');
            return;
        }
        
        // Clear existing markers first
        //clearAllMarkers();
        
        m = markerData[markerData.length - 1];
        if (!m.lat || !m.lng) {
            console.warn('Skipping marker with missing lat/lng:', m);
            return;
        }
        
        const marker = L.marker([m.lat, m.lng]).addTo(map);
        
        const popupContent = `
            <div class="marker-popup">
                <h4>Marker Details</h4>
                <p><strong>ID:</strong> ${m.id || 'N/A'}</p>
                <p><strong>Latitude:</strong> ${m.lat.toFixed(6)}</p>
                <p><strong>Longitude:</strong> ${m.lng.toFixed(6)}</p>
                <p><strong>Recorded:</strong> ${m.timestamp ? new Date(m.timestamp).toLocaleString() : 'N/A'}</p>
            </div>
        `;
        
        marker.bindPopup(popupContent);
        
        markers.push({
            id: m.id || `MARKER-${String(markerCounter).padStart(4, '0')}`,
            lat: m.lat,
            lng: m.lng,
            timestamp: m.timestamp || new Date().toISOString(),
            leafletMarker: marker
        });
        
        markerCounter++;
        
        
        // Fit map to show all markers
        if (markers.length > 0) {
            const group = L.featureGroup(markers.map(m => m.leafletMarker));
            map.fitBounds(group.getBounds().pad(0.1));
        }
        
        // Save to localStorage
        saveMarkers();
        
        console.log(`Successfully loaded ${markers.length} markers from ${filename}`);
    } catch (error) {
        console.error('Error loading JSON file:', error);
        alert('Error loading JSON file: ' + error.message);
    }
}

async function loadAllMarkerFiles(maxFileNumber = 255) {
    let filesLoaded = 0;
    
    for (let i = 0; i <= maxFileNumber; i++) {
        try {
            const response = await fetch(`GPSData/${i}.json`);
            if (response.ok) {
                await loadMarkersFromFile(`${i}.json`);
                filesLoaded++;
                console.log(`Loaded ${i}.json`);
            }
        } catch (error) {
            // File doesn't exist or error loading, continue to next
            console.log(`Skipping ${i}.json (not found or error)`);
        }
    }
    
    console.log(`Finished loading: ${filesLoaded} files loaded, ${markers.length} total markers`);
}

// Load markers from JSON file upload
// async function loadMarkersFromJSON(file) {
//     try {
//         const text = await file.text();
//         const markerData = JSON.parse(text);
        
//         if (!Array.isArray(markerData)) {
//             alert('JSON file must contain an array of markers');
//             return;
//         }
        
//         // Clear existing markers first
//         clearAllMarkers();
        

//         m = markerData[markerData.length - 1];


//         //m = markerData.pop()[-1];

//         if (!m.lat || !m.lng) {
//             console.warn('Skipping marker with missing lat/lng:', m);
//             return;
//         }
        
//         const marker = L.marker([m.lat, m.lng]).addTo(map);
        
//         const popupContent = `
//             <div class="marker-popup">
//                 <h4>Marker Details</h4>
//                 <p><strong>ID:</strong> ${m.id || 'N/A'}</p>
//                 <p><strong>Latitude:</strong> ${m.lat.toFixed(6)}</p>
//                 <p><strong>Longitude:</strong> ${m.lng.toFixed(6)}</p>
//                 <p><strong>Recorded:</strong> ${m.timestamp ? new Date(m.timestamp).toLocaleString() : 'N/A'}</p>
//             </div>
//         `;
        
//             marker.bindPopup(popupContent);
            
//             markers.push({
//                 id: m.id || `MARKER-${String(markerCounter).padStart(4, '0')}`,
//                 lat: m.lat,
//                 lng: m.lng,
//                 timestamp: m.timestamp || new Date().toISOString(),
//                 leafletMarker: marker
//             });
            
//             markerCounter++;
    
        
//         // Fit map to show all markers
//         if (markers.length > 0) {
//             const group = L.featureGroup(markers.map(m => m.leafletMarker));
//             map.fitBounds(group.getBounds().pad(0.1));
//         }
        
//         // Save to localStorage
//         saveMarkers();
        
//         alert(`Successfully loaded ${markers.length} markers`);
//     } catch (error) {
//         alert('Error loading JSON file: ' + error.message);
//         console.error('JSON load error:', error);
//     }
// }



// Event listeners
document.getElementById('addMarker').addEventListener('click', () => {
    const lat = parseFloat(document.getElementById('latitude').value);
    const lng = parseFloat(document.getElementById('longitude').value);
    
    if (isNaN(lat) || isNaN(lng)) {
        alert('Please enter valid latitude and longitude values');
        return;
    }
    
    if (lat < -90 || lat > 90) {
        alert('Latitude must be between -90 and 90');
        return;
    }
    
    if (lng < -180 || lng > 180) {
        alert('Longitude must be between -180 and 180');
        return;
    }
    
    addMarker(lat, lng);
    
    // Clear inputs
    document.getElementById('latitude').value = '';
    document.getElementById('longitude').value = '';
});

document.getElementById('clearMarkers').addEventListener('click', () => {
    if (markers.length === 0) {
        alert('No markers to clear');
        return;
    }
    
    if (confirm('Are you sure you want to clear all markers?')) {
        clearAllMarkers();
    }
});

// Allow Enter key to add marker
document.getElementById('longitude').addEventListener('keypress', (e) => {
    if (e.key === 'Enter') {
        document.getElementById('addMarker').click();
    }
});

// Load saved markers on page load
loadMarkers();
loadAllMarkerFiles();
