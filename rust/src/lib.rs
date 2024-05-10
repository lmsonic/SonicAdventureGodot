use godot::prelude::*;
mod character_body_3d;
struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension {}
