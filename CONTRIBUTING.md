# Cómo colaborar

Este repositorio lo mantienen Reymildo y Montse. Ambos son **colaboradores con permiso de escritura (Write)** directamente sobre el repositorio, así que los dos pueden:

- Clonar el repo y hacer `git push` directamente.
- Crear ramas (`git checkout -b nombre-de-la-rama`) para trabajar en features sin afectar `main`.
- Abrir Pull Requests para revisar cambios grandes antes de fusionarlos (recomendado para firmware/lógica de misión), o hacer push directo a `main` para cambios pequeños (docs, ajustes menores).

## Flujo sugerido

1. `git pull` antes de empezar a trabajar, para tener la última versión.
2. Trabajar en una rama si el cambio es grande: `git checkout -b feature/nombre`.
3. Hacer commits descriptivos en español o inglés, como prefieran, pero consistentes.
4. `git push origin feature/nombre` y abrir un Pull Request en GitHub, o hacer merge directo si ya lo hablaron.

## Configurar Git por primera vez

```bash
git config --global user.name "Tu Nombre"
git config --global user.email "tu-correo@ejemplo.com"
```

## Clonar el repositorio

```bash
git clone https://github.com/reymenez323/Athena-Rover-2026.git
cd Athena-Rover-2026
```

## Trabajando desde el mismo chat de Claude

Si ambos comparten la misma sesión de Claude (Cowork), cualquiera puede pedirle a Claude que haga cambios, commits y push al repo — Claude usará las credenciales de GitHub configuradas en esa sesión. No es necesario que cada quien tenga Git configurado localmente si todo el trabajo se hace a través de Claude, pero sigue siendo buena idea que cada quien tenga su propia cuenta de GitHub como colaborador por si necesitan trabajar de forma independiente (por ejemplo, subiendo cambios desde su propia laptop con VS Code / Arduino IDE / PlatformIO).
