# butia_world

Implements plugins that manage and filter information coming from sensors to a centralized world representation

### Plugins

- **boiler_plate_writer**
    
    Writes a counter in the *heartbeat* key every 1 sec.

- **boiler_plate_reader**
    
    Reads from the *heartbeat* key every 1 sec and publishes in the
    */world/heartbeat* topic.