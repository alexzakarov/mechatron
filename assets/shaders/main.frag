#version 330 core
in vec3 v_normal;
in vec3 v_frag_pos;

uniform vec3 u_color;
uniform vec3 u_light_dir;

out vec4 frag_color;

void main() {
    vec3 norm = normalize(v_normal);
    float diff = max(dot(norm, normalize(u_light_dir)), 0.0);
    vec3 ambient = 0.3 * u_color;
    vec3 diffuse = 0.7 * diff * u_color;
    frag_color = vec4(ambient + diffuse, 1.0);
}
