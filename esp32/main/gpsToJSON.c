#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include "json_object.h"
#include "json_tokener.h"
#include "gps.h"

// Flags to indicate which fields are present in memory
typedef enum {
    COORD_HAS_ID        = (1 << 0),
    COORD_HAS_LAT       = (1 << 1),
    COORD_HAS_LON       = (1 << 2),
    COORD_HAS_TIMESTAMP = (1 << 3)
} CoordFieldFlags;

int find_or_create_json(const char *folder_path, const char *json_filename) {
    char filepath[512];
    
    // Construct the full file path
    snprintf(filepath, sizeof(filepath), "%s/%s", folder_path, json_filename);
    
    // Check if file exists
    FILE *file = fopen(filepath, "r");
    if (file != NULL) {
        printf("JSON file '%s' found in folder '%s'.\n", json_filename, folder_path);
        fclose(file);
        return 0;
    }
    
    // File doesn't exist, create it
    file = fopen(filepath, "w");
    if (file == NULL) {
        perror("Error creating JSON file");
        return -1;
    }
    
    // Write empty JSON object
    fprintf(file, "{}");
    fclose(file);
    
    printf("JSON file '%s' created in folder '%s'.\n", json_filename, folder_path);
    return 0;
}

// Function to create a folder in the local directory
// Returns 0 on success, -1 on failure
int create_folder(const char *folder_name) {
    struct stat st = {0};
    
    // Check if folder already exists
    if (stat(folder_name, &st) == 0 && S_ISDIR(st.st_mode)) {
        // printf("Folder '%s' already exists. Nothing to do.\n", folder_name);
        return 1;
    }
    
    #ifdef _WIN32
        // Windows
        if (mkdir(folder_name) == 0) {
            // printf("Folder '%s' created successfully.\n", folder_name);
            return 0;
        }
    #else
        // Unix/Linux/Mac - requires permissions parameter
        if (mkdir
            (folder_name, 0755) == 0) {
            // printf("Folder '%s' created successfully.\n", folder_name);
            return 0;
        }
    #endif
    
    perror("Error creating folder");
    return -1;
}

// Read coordinates from memory, convert to lat/lon, and write to JSON file
// Missing fields will be added as null entries in the JSON
int write_coordinates_to_json(const void *ptr, size_t length, int id, unsigned int field_flags) {
    if (!ptr) {
        fprintf(stderr, "Invalid parameters\n");
        return -1;
    }

    // Create folder
    const char *GPSFolderPath = "../../GPSData";
    create_folder(GPSFolderPath);

    // Build GPS id string
    char GPSid[64];
    snprintf(GPSid, sizeof(GPSid), "%d.json", id);   // file is "1.json", "2.json", etc.

    // Build full path to the JSON file
    char json_filename[512];
    snprintf(json_filename, sizeof(json_filename), "%s/%s", GPSFolderPath, GPSid);

    // Ensure the JSON file exists
    find_or_create_json(GPSFolderPath, GPSid);

    
    
    // Read coordinate data from memory based on what's available
    gnss_data_t coord = {0};
    coord.id = id;
    
    size_t offset = 0;
    const unsigned char *data = (const unsigned char *)ptr;
    
    // Read fields in order based on flags
    if (field_flags & COORD_HAS_ID && offset + sizeof(int) <= length) {
        memcpy(&coord.id, data + offset, sizeof(int));
        offset += sizeof(int);
    }
    
    if (field_flags & COORD_HAS_LAT && offset + sizeof(double) <= length) {
        memcpy(&coord.latitude, data + offset, sizeof(double));
        offset += sizeof(double);
    }
    
    if (field_flags & COORD_HAS_LON && offset + sizeof(double) <= length) {
        memcpy(&coord.longitude, data + offset, sizeof(double));
        offset += sizeof(double);
    }
    
    if (field_flags & COORD_HAS_TIMESTAMP && offset + sizeof(long) <= length) {
        memcpy(&coord.timestamp, data + offset, sizeof(long));
        offset += sizeof(long);
    }
    
    // Read existing JSON file or create new root object
    struct json_object *root = NULL;
    FILE *fp = fopen(json_filename, "r");
    
    if (fp) {
        // File exists, parse it
        fseek(fp, 0, SEEK_END);
        long fsize = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        
        char *file_content = malloc(fsize + 1);
        if (!file_content) {
            fclose(fp);
            return -1;
        }
        
        fread(file_content, 1, fsize, fp);
        file_content[fsize] = '\0';
        fclose(fp);
        
        // Parse existing JSON
        root = json_tokener_parse(file_content);
        free(file_content);
        
        if (!root) {
            fprintf(stderr, "Failed to parse existing JSON file\n");
            root = json_object_new_object();
        }
    } else {
        // Create new JSON object
        root = json_object_new_object();
        if (!root) {
            fprintf(stderr, "Failed to create JSON object\n");
            return -1;
        }
    }
    
    // Create coordinate entry with null for missing fields
    struct json_object *coord_obj = json_object_new_object();
    if (!coord_obj) {
        json_object_put(root);
        return -1;
    }
    
    // Add ID (always present)
    json_object_object_add(coord_obj, "id", json_object_new_int(coord.id));
    
    // Add latitude or null
    if (field_flags & COORD_HAS_LAT) {
        json_object_object_add(coord_obj, "latitude", json_object_new_double(coord.latitude));
    } else {
        json_object_object_add(coord_obj, "latitude", NULL);
    }
    
    // Add longitude or null
    if (field_flags & COORD_HAS_LON) {
        json_object_object_add(coord_obj, "longitude", json_object_new_double(coord.longitude));
    } else {
        json_object_object_add(coord_obj, "longitude", NULL);
    }
    
    // Add timestamp or null
    if (field_flags & COORD_HAS_TIMESTAMP) {
        json_object_object_add(coord_obj, "timestamp", json_object_new_int64(coord.timestamp));
    } else {
        json_object_object_add(coord_obj, "timestamp", NULL);
    }
    
    // Create key string from ID
    char key[32];
    snprintf(key, sizeof(key), "%d", id);
    
    // Check if entry exists and update or add new
    struct json_object *existing = NULL;
    if (json_object_object_get_ex(root, key, &existing)) {
        // Entry exists, delete old one
        json_object_object_del(root, key);
    }
    
    // Add new/updated entry
    json_object_object_add(root, key, coord_obj);
    
    // Write to file with pretty formatting
    const char *json_string = json_object_to_json_string_ext(root, 
                                                             JSON_C_TO_STRING_PRETTY);
    
    fp = fopen(json_filename, "w");
    if (!fp) {
        fprintf(stderr, "Failed to open file for writing: %s\n", strerror(errno));
        json_object_put(root);
        return -1;
    }
    
    fprintf(fp, "%s\n", json_string);
    fclose(fp);
    
    // Clean up
    json_object_put(root);
    
    printf("Successfully wrote coordinate (ID: %d) to %s\n", coord.id, json_filename);
    
    return 0;
}

/**
 * Alternative: Read raw bytes and interpret as lat/lon pairs
 * Assumes memory layout: [double latitude][double longitude]
 * Missing optional fields will be set to null
 */
int write_raw_coordinates_to_json(const void *ptr, size_t length, int id, 
                                 unsigned int field_flags, const char *json_filename) {
    if (!ptr || !json_filename) {
        fprintf(stderr, "Invalid parameters or insufficient data\n");
        return -1;
    }

    // Read fields from memory based on flags
    double latitude = 0.0, longitude = 0.0;
    long timestamp = 0;
    
    size_t offset = 0;
    const unsigned char *data = (const unsigned char *)ptr;
    
    if (field_flags & COORD_HAS_LAT && offset + sizeof(double) <= length) {
        memcpy(&latitude, data + offset, sizeof(double));
        offset += sizeof(double);
    }
    
    if (field_flags & COORD_HAS_LON && offset + sizeof(double) <= length) {
        memcpy(&longitude, data + offset, sizeof(double));
        offset += sizeof(double);
    }
    
    if (field_flags & COORD_HAS_TIMESTAMP && offset + sizeof(long) <= length) {
        memcpy(&timestamp, data + offset, sizeof(long));
        offset += sizeof(long);
    }
    
    // Read existing JSON file or create new root object
    struct json_object *root = NULL;
    FILE *fp = fopen(json_filename, "r");
    
    if (fp) {
        fseek(fp, 0, SEEK_END);
        long fsize = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        
        char *file_content = malloc(fsize + 1);
        if (!file_content) {
            fclose(fp);
            return -1;
        }
        
        fread(file_content, 1, fsize, fp);
        file_content[fsize] = '\0';
        fclose(fp);
        
        root = json_tokener_parse(file_content);
        free(file_content);
        
        if (!root) {
            root = json_object_new_object();
        }
    } else {
        root = json_object_new_object();
        if (!root) {
            return -1;
        }
    }
    
    // Create coordinate entry with null for missing fields
    struct json_object *coord_obj = json_object_new_object();
    if (!coord_obj) {
        json_object_put(root);
        return -1;
    }
    
    json_object_object_add(coord_obj, "id", json_object_new_int(id));
    
    // Add fields or null
    if (field_flags & COORD_HAS_LAT) {
        json_object_object_add(coord_obj, "latitude", json_object_new_double(latitude));
    } else {
        json_object_object_add(coord_obj, "latitude", NULL);
    }
    
    if (field_flags & COORD_HAS_LON) {
        json_object_object_add(coord_obj, "longitude", json_object_new_double(longitude));
    } else {
        json_object_object_add(coord_obj, "longitude", NULL);
    }
    
    if (field_flags & COORD_HAS_TIMESTAMP) {
        json_object_object_add(coord_obj, "timestamp", json_object_new_int64(timestamp));
    } else {
        json_object_object_add(coord_obj, "timestamp", NULL);
    }
    
    // Create key from ID
    char key[32];
    snprintf(key, sizeof(key), "%d", id);
    
    // Update or add entry
    struct json_object *existing = NULL;
    if (json_object_object_get_ex(root, key, &existing)) {
        json_object_object_del(root, key);
    }
    
    json_object_object_add(root, key, coord_obj);
    
    // Write to file
    const char *json_string = json_object_to_json_string_ext(root, 
                                                             JSON_C_TO_STRING_PRETTY);
    
    fp = fopen(json_filename, "w");
    if (!fp) {
        fprintf(stderr, "Failed to open file for writing\n");
        json_object_put(root);
        return -1;
    }
    
    fprintf(fp, "%s\n", json_string);
    fclose(fp);
    json_object_put(root);
    
    printf("Successfully wrote coordinate (ID: %d) to %s\n", id, json_filename);
    
    return 0;
}


/**
    // Example 1: Full coordinate data with all fields
    Coordinate coord1 = {1, 40.7128, -74.0060, 1699564800}; // New York with timestamp
    unsigned int full_flags = COORD_HAS_ID | COORD_HAS_LAT | COORD_HAS_LON | COORD_HAS_TIMESTAMP;
    write_coordinates_to_json(&coord1, sizeof(coord1), 1, full_flags, "coordinates.json");
    
    // Example 2: Coordinate with missing timestamp (will be null in JSON)
    struct {
        int id;
        double lat;
        double lon;
    } coord2_partial = {2, 34.0522, -118.2437}; // Los Angeles, no timestamp
    
    unsigned int partial_flags = COORD_HAS_ID | COORD_HAS_LAT | COORD_HAS_LON;
    // Note: timestamp will be null in JSON
    write_coordinates_to_json(&coord2_partial, sizeof(coord2_partial), 2, partial_flags, "coordinates.json");
    
    // Example 3: Only lat/lon, everything else null
    double coords3[] = {51.5074, -0.1278}; // London
    unsigned int latlon_only = COORD_HAS_LAT | COORD_HAS_LON;
    write_raw_coordinates_to_json(coords3, sizeof(coords3), 3, latlon_only, "coordinates.json");
    
    // Example 4: Coordinate with timestamp
    struct {
        double lat;
        double lon;
        long ts;
    } coord4 = {48.8566, 2.3522, 1699651200}; // Paris
    
    unsigned int with_timestamp = COORD_HAS_LAT | COORD_HAS_LON | COORD_HAS_TIMESTAMP;
    write_raw_coordinates_to_json(&coord4, sizeof(coord4), 4, with_timestamp, "coordinates.json");
    
    // Example 5: Update existing entry, adding timestamp that was missing
    Coordinate coord2_updated = {2, 34.0522, -118.2437, 1699737600};
    write_coordinates_to_json(&coord2_updated, sizeof(coord2_updated), 2, full_flags, "coordinates.json");
    
    printf("\nJSON file 'coordinates.json' has been created/updated.\n");
    printf("Entries with missing data will show 'null' in those fields.\n");
    
    return 0;
*/