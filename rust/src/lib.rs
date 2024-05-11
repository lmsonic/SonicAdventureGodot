use godot::prelude::*;
mod character_body_3d;
mod motion_structs;
struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension {}
