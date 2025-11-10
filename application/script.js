const image = document.querySelector('.opacity-image');
const markerDetails = document.getElementById('marker-details');

// GPS coordinates of the four corners of the image
const corners = [
    top_left = (38.03524477143201, -78.51456138025434),
    top_right = (38.034805350705405, -78.50856396115098),
    bottom_right = (38.03163992670867, -78.50923156738281),
    bottom_left = (38.03207934643092, -78.51522898674011),
];

// Function to convert GPS coordinates to pixel coordinates
function gpsToPixel(lat, lon){
    const topBorder = corners.top_left[0];
    const botBorder = corners.bottom_left[0];
    const leftBorder = corners.top_left[1];
    const rightBorder = corners.top_right[1];

    // const topBorder = 38.03524477143201;
    // const botBorder = 38.03163992670867;
    // const leftBorder = -78.51456138025434;
    // const rightBorder = -78.50856396115098;

    const relativeX = (lon - leftBorder) / (rightBorder - leftBorder);
    const relativeY = (lat - topBorder) / (botBorder - topBorder);

    const imgRect = image.getBoundingClientRect();
    const wrapperRect = image.parentElement.getBoundingClientRect();
    
    const offsetX = imgRect.left - wrapperRect.left;
    const offsetY = imgRect.top - wrapperRect.top;
    
    const pixelX = offsetX + (relativeX * imgRect.width);
    const pixelY = offsetY + (relativeY * imgRect.height);

    return [pixelX, pixelY];
}

// Function to plot a marker on the image based on GPS coordinates
function createMarker(id, lat, lon, timestamp) {
    const position = gpsToPixel(lat, lon);

    const marker = document.createElement('div');
    marker.classList.add('marker');
    marker.id = id;
    marker.style.left = position[0] + 'px';
    marker.style.top = position[1] + 'px';
    marker.style.display = 'block';

    marker.addEventListener('click', function(e) {
        e.stopPropagation();
        showDetails(id, lat, lon, timestamp);
    });
    
    image.parentElement.appendChild(marker);
    return marker;
}




function showDetails(id, lat, lon, timestamp) {
    markerDetails.style.display = 'block';
    markerDetails.innerHTML = `
        <h2>Marker Details</h2>
        <p>ID: ${id}</p>
        <p>Latitude: ${lat}</p>
        <p>Longitude: ${lon}</p>
        <p>Timestamp: ${timestamp}</p>
    `;
}

function hideDetails() {
    infoPopup.style.display = 'none';
}

document.addEventListener('click', hideDetails);

function createExampleMarkers() {
    createMarker('Center', 38.03344, -78.51190, 1124032021);
}


// Plot a sample coordinate when the image loads
image.addEventListener('load', createExampleMarkers);


window.addEventListener('resize', function() {
    const existingMarkers = document.querySelectorAll('.marker');
    existingMarkers.forEach(m => m.remove());
    // adjust marker position on window resize
    plotCoordinate(38.03344, -78.51190);
    
    createExampleMarkers();
});

if (image.complete) {
    createExampleMarkers();
}