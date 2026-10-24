/**
 * SwarmTech3_UpRising: Three.js WebGL Tactical Viewport (app.js)
 * 100% GPU-accelerated 3D rendering (zero CPU software projection).
 * True 3D quadcopters, spinning rotors, perspective drop-lines,
 * concentric range rings, and dynamic glowing catenary tether.
 */

(function () {
    'use strict';

    // =========================================================================
    // Configuration & Visual Constants
    // =========================================================================
    const COLORS = {
        bg: 0x061019,
        grid: 0x0B2A3D,
        gridHighlight: 0x00F0FF,
        ring25: 0x00F0FF,
        ring50: 0x00F0FF,
        ring100: 0x0088AA,
        leader: 0xFFB800,
        escort: 0x00F0FF,
        crashed: 0xFF3366,
        propagating: 0xFFB800,
        tether: 0x00F0FF,
        target: 0x00FFAA,
        dropLine: 0x00F0FF,
    };

    let scene, camera, renderer, controls;
    let container;
    let dronesMap = new Map();
    let tetherMesh = null;
    let targetBeacon = null;
    let gcsMast = null;

    let isChaseCam = false;
    let targetDroneId = 0;
    let lastTelemetryTime = 0;
    let fpsCounter = 0;
    let lastFpsUpdate = 0;

    // =========================================================================
    // 1. Scene & Renderer Initialization
    // =========================================================================
    function init() {
        container = document.getElementById('viewport-container');

        // 1. Three.js Scene with Cyber Fog
        scene = new THREE.Scene();
        scene.background = new THREE.Color(COLORS.bg);
        scene.fog = new THREE.FogExp2(COLORS.bg, 0.0035);

        // 2. Camera Setup (Z-Up aerospace coordinate standard)
        camera = new THREE.PerspectiveCamera(
            55,
            window.innerWidth / window.innerHeight,
            0.5,
            1200
        );
        camera.up.set(0, 0, 1);
        camera.position.set(70, -70, 65);

        // 3. WebGL Renderer with High-DPI Support
        renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'high-performance' });
        renderer.setSize(window.innerWidth, window.innerHeight);
        renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        renderer.toneMapping = THREE.ACESFilmicToneMapping;
        renderer.toneMappingExposure = 1.2;
        container.appendChild(renderer.domElement);

        // 4. Orbit Controls
        controls = new THREE.OrbitControls(camera, renderer.domElement);
        controls.enableDamping = true;
        controls.dampingFactor = 0.06;
        controls.screenSpacePanning = false;
        controls.minDistance = 5;
        controls.maxDistance = 500;
        controls.maxPolarAngle = Math.PI / 2 - 0.02; // Don't go below floor
        controls.target.set(30, 20, 15);

        // 5. Lighting
        const ambientLight = new THREE.AmbientLight(0x1B2C3D, 1.2);
        scene.add(ambientLight);

        const dirLight = new THREE.DirectionalLight(0xFFFFFF, 1.5);
        dirLight.position.set(60, 40, 100);
        scene.add(dirLight);

        const blueLight = new THREE.PointLight(0x00F0FF, 1.8, 150);
        blueLight.position.set(0, 0, 30);
        scene.add(blueLight);

        // 6. Build Tactical Environment
        createGroundGrid();
        createTacticalRangeRings();
        createGcsMast();
        createTargetBeacon();
        createCatenaryTether();

        // 7. Event Listeners
        window.addEventListener('resize', onWindowResize);
        window.addEventListener('keydown', onKeyDown);
        renderer.domElement.addEventListener('pointerdown', onPointerDown);
        renderer.domElement.addEventListener('pointerup', onPointerUp);

        // 8. Connect WebSocket Telemetry
        if (window.swarmClient) {
            window.swarmClient.on('telemetry', onTelemetryFrame);
            window.swarmClient.on('status', onConnectionStatus);
        }

        animate();
    }

    // =========================================================================
    // 2. Tactical Ground Environment & Grid
    // =========================================================================
    function createGroundGrid() {
        // Grid on Z=0 plane (XY)
        const size = 300;
        const divisions = 60;
        const grid = new THREE.GridHelper(size, divisions, COLORS.gridHighlight, COLORS.grid);
        grid.rotation.x = Math.PI / 2;
        grid.position.set(0, 0, 0);
        scene.add(grid);
    }

    function createTacticalRangeRings() {
        const radii = [25, 50, 100];
        radii.forEach((r, idx) => {
            const curve = new THREE.EllipseCurve(
                0, 0, r, r, 0, 2 * Math.PI, false, 0
            );
            const points = curve.getPoints(120);
            const geometry = new THREE.BufferGeometry().setFromPoints(
                points.map(p => new THREE.Vector3(p.x, p.y, 0.05))
            );
            const material = new THREE.LineDashedMaterial({
                color: idx === 2 ? COLORS.ring100 : COLORS.ring25,
                dashSize: 3,
                gapSize: 2,
                linewidth: 1,
                transparent: true,
                opacity: 0.65,
            });
            const line = new THREE.Line(geometry, material);
            line.computeLineDistances();
            scene.add(line);
        });
    }

    function createGcsMast() {
        gcsMast = new THREE.Group();

        // Base pad
        const baseGeom = new THREE.CylinderGeometry(2.5, 3.0, 0.6, 16);
        baseGeom.rotateX(Math.PI / 2);
        const baseMat = new THREE.MeshStandardMaterial({
            color: 0x0A1C29,
            metalness: 0.8,
            roughness: 0.3,
        });
        const baseMesh = new THREE.Mesh(baseGeom, baseMat);
        baseMesh.position.set(0, 0, 0.3);
        gcsMast.add(baseMesh);

        // Vertical mast
        const mastGeom = new THREE.CylinderGeometry(0.15, 0.25, 1.5, 8);
        mastGeom.rotateX(Math.PI / 2);
        const mastMat = new THREE.MeshStandardMaterial({
            color: 0x5A7E96,
            metalness: 0.9,
            roughness: 0.2,
        });
        const mastMesh = new THREE.Mesh(mastGeom, mastMat);
        mastMesh.position.set(0, 0, 0.75);
        gcsMast.add(mastMesh);

        // Flashing anchor beacon
        const beaconGeom = new THREE.SphereGeometry(0.35, 16, 16);
        const beaconMat = new THREE.MeshBasicMaterial({
            color: COLORS.leader,
            transparent: true,
            opacity: 0.9,
        });
        const beaconMesh = new THREE.Mesh(beaconGeom, beaconMat);
        beaconMesh.position.set(0, 0, 1.5);
        gcsMast.add(beaconMesh);

        scene.add(gcsMast);
    }

    function createTargetBeacon() {
        targetBeacon = new THREE.Group();

        // 1. Rotating diamond
        const octGeom = new THREE.OctahedronGeometry(1.2, 0);
        const octMat = new THREE.MeshStandardMaterial({
            color: COLORS.target,
            emissive: COLORS.target,
            emissiveIntensity: 0.8,
            wireframe: true,
        });
        const diamond = new THREE.Mesh(octGeom, octMat);
        diamond.position.set(0, 0, 0);
        diamond.name = 'diamond';
        targetBeacon.add(diamond);

        // 2. Vertical laser beam to ground
        const beamGeom = new THREE.CylinderGeometry(0.08, 0.08, 60, 8);
        beamGeom.rotateX(Math.PI / 2);
        const beamMat = new THREE.MeshBasicMaterial({
            color: COLORS.target,
            transparent: true,
            opacity: 0.35,
        });
        const beam = new THREE.Mesh(beamGeom, beamMat);
        beam.position.set(0, 0, -30);
        targetBeacon.add(beam);

        // 3. Ground pulse ring
        const ringGeom = new THREE.RingGeometry(1.8, 2.2, 32);
        const ringMat = new THREE.MeshBasicMaterial({
            color: COLORS.target,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.6,
        });
        const ring = new THREE.Mesh(ringGeom, ringMat);
        ring.name = 'ring';
        targetBeacon.add(ring);

        targetBeacon.position.set(60, 40, 30);
        scene.add(targetBeacon);
    }

    function createCatenaryTether() {
        // Dynamic curve tube
        const curve = new THREE.LineCurve3(
            new THREE.Vector3(0, 0, 1.5),
            new THREE.Vector3(0, 0, 20)
        );
        const geom = new THREE.BufferGeometry().setFromPoints(curve.getPoints(16));
        const mat = new THREE.LineBasicMaterial({
            color: COLORS.tether,
            linewidth: 2,
            transparent: true,
            opacity: 0.85,
        });
        tetherMesh = new THREE.Line(geom, mat);
        scene.add(tetherMesh);
    }

    // =========================================================================
    // 3. True 3D Quadcopter Mesh Builder
    // =========================================================================
    function buildQuadcopterMesh(isLeader) {
        const droneGroup = new THREE.Group();
        const scale = isLeader ? 1.15 : 1.0;

        // 1. Central fuselage body
        const bodyGeom = new THREE.BoxGeometry(0.55 * scale, 0.45 * scale, 0.22 * scale);
        const bodyMat = new THREE.MeshStandardMaterial({
            color: 0x111D28,
            metalness: 0.85,
            roughness: 0.25,
        });
        const bodyMesh = new THREE.Mesh(bodyGeom, bodyMat);
        droneGroup.add(bodyMesh);

        // Glowing center core
        const coreGeom = new THREE.SphereGeometry(0.18 * scale, 12, 12);
        const coreMat = new THREE.MeshBasicMaterial({
            color: isLeader ? COLORS.leader : COLORS.escort,
        });
        const coreMesh = new THREE.Mesh(coreGeom, coreMat);
        coreMesh.name = 'core';
        droneGroup.add(coreMesh);

        // 2. Four Diagonal Carbon Arms
        const armLength = 0.75 * scale; // wingspan ~1.5m
        const armGeom = new THREE.CylinderGeometry(0.04, 0.04, armLength, 8);
        armGeom.rotateZ(Math.PI / 2);
        const armMat = new THREE.MeshStandardMaterial({
            color: 0x243542,
            metalness: 0.9,
            roughness: 0.3,
        });

        // Arm 1 & 2 (X configuration)
        const armA = new THREE.Mesh(armGeom, armMat);
        armA.rotation.z = Math.PI / 4;
        droneGroup.add(armA);

        const armB = new THREE.Mesh(armGeom, armMat);
        armB.rotation.z = -Math.PI / 4;
        droneGroup.add(armB);

        // 3. Motor Pods & Spinning Rotor Discs
        const rotors = [];
        const armOffset = (armLength / 2) * Math.SQRT1_2;
        const motorPositions = [
            [armOffset, armOffset],
            [-armOffset, armOffset],
            [armOffset, -armOffset],
            [-armOffset, -armOffset],
        ];

        const motorGeom = new THREE.CylinderGeometry(0.07, 0.07, 0.16, 8);
        motorGeom.rotateX(Math.PI / 2);
        const motorMat = new THREE.MeshStandardMaterial({ color: 0x334455, metalness: 0.8 });

        const rotorGeom = new THREE.CircleGeometry(0.38 * scale, 16);
        const rotorMat = new THREE.MeshBasicMaterial({
            color: isLeader ? COLORS.leader : COLORS.escort,
            transparent: true,
            opacity: 0.45,
            side: THREE.DoubleSide,
        });

        motorPositions.forEach((pos, idx) => {
            const motor = new THREE.Mesh(motorGeom, motorMat);
            motor.position.set(pos[0], pos[1], 0.08 * scale);
            droneGroup.add(motor);

            // Rotor Disc
            const rotor = new THREE.Mesh(rotorGeom, rotorMat.clone());
            rotor.position.set(pos[0], pos[1], 0.16 * scale);
            droneGroup.add(rotor);
            rotors.push(rotor);
        });

        droneGroup.userData.rotors = rotors;

        // 4. Orientation LED indicator (Front = Cyan/Green, Rear = Red)
        const fwdLedGeom = new THREE.SphereGeometry(0.06, 8, 8);
        const fwdLedMat = new THREE.MeshBasicMaterial({ color: 0x00FF88 });
        const fwdLed = new THREE.Mesh(fwdLedGeom, fwdLedMat);
        fwdLed.position.set(0.32 * scale, 0, 0);
        droneGroup.add(fwdLed);

        return droneGroup;
    }

    function createDropLine(isLeader) {
        const dropLineGeom = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(0, 0, 0),
            new THREE.Vector3(0, 0, 0),
        ]);
        const dropLineMat = new THREE.LineDashedMaterial({
            color: isLeader ? COLORS.leader : COLORS.dropLine,
            dashSize: 1.0,
            gapSize: 0.8,
            linewidth: 1,
            transparent: true,
            opacity: 0.5,
        });
        const dropLine = new THREE.Line(dropLineGeom, dropLineMat);
        scene.add(dropLine);
        return dropLine;
    }

    function createGroundShadow(isLeader) {
        const scale = isLeader ? 1.15 : 1.0;
        const shadowGeom = new THREE.RingGeometry(0.8 * scale, 1.0 * scale, 16);
        const shadowMat = new THREE.MeshBasicMaterial({
            color: isLeader ? COLORS.leader : COLORS.dropLine,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.45,
        });
        const shadowMesh = new THREE.Mesh(shadowGeom, shadowMat);
        scene.add(shadowMesh);
        return shadowMesh;
    }

    // 3D Billboard Tactical ID Badge Floating Above Drone Airframe
    function createDroneBadge(droneId, isLeader) {
        const canvas = document.createElement('canvas');
        canvas.width = 256;
        canvas.height = 64;
        const texture = new THREE.CanvasTexture(canvas);
        texture.minFilter = THREE.LinearFilter;

        const spriteMat = new THREE.SpriteMaterial({
            map: texture,
            transparent: true,
            depthTest: false,
        });
        const sprite = new THREE.Sprite(spriteMat);
        sprite.scale.set(2.4, 0.6, 1.0);
        sprite.userData = { canvas, texture, lastKey: '' };
        scene.add(sprite);
        return sprite;
    }

    function updateDroneBadge(sprite, d) {
        if (!sprite || !sprite.userData) return;
        const { canvas, texture } = sprite.userData;
        const key = `${d.id}_${d.is_leader}_${d.crashed}_${d.lost_link}_${d.status}`;
        if (sprite.userData.lastKey === key) return;
        sprite.userData.lastKey = key;

        const ctx = canvas.getContext('2d');
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // Cyberpunk tactical styling
        let borderColor = '#00F0FF';
        let bgColor = 'rgba(6, 16, 25, 0.88)';
        let textColor = '#00F0FF';
        let labelText = `#${d.id}`;
        let subText = 'ESCORT';
        let dotColor = '#00F0FF';

        if (d.crashed) {
            borderColor = '#FF3366';
            bgColor = 'rgba(45, 10, 20, 0.92)';
            textColor = '#FF3366';
            labelText = `#${d.id}`;
            subText = 'CRASHED';
            dotColor = '#FF3366';
        } else if (d.is_leader) {
            borderColor = '#FFB800';
            bgColor = 'rgba(42, 28, 4, 0.92)';
            textColor = '#FFB800';
            labelText = `#${d.id}`;
            subText = 'FLAGSHIP';
            dotColor = '#FFB800';
        } else if (d.lost_link) {
            borderColor = '#FFB800';
            bgColor = 'rgba(38, 26, 5, 0.92)';
            textColor = '#FFD24D';
            labelText = `#${d.id}`;
            subText = 'LOST LINK';
            dotColor = '#FFB800';
        } else if (d.status === 'PROPAGATING') {
            borderColor = '#00FFAA';
            bgColor = 'rgba(4, 32, 20, 0.92)';
            textColor = '#00FFAA';
            labelText = `#${d.id}`;
            subText = 'ECHO';
            dotColor = '#00FFAA';
        }

        // Draw rounded tactical card
        const x = 4, y = 4, w = 248, h = 56, r = 8;
        ctx.beginPath();
        ctx.moveTo(x + r, y);
        ctx.lineTo(x + w - r, y);
        ctx.quadraticCurveTo(x + w, y, x + w, y + r);
        ctx.lineTo(x + w, y + h - r);
        ctx.quadraticCurveTo(x + w, y + h, x + w - r, y + h);
        ctx.lineTo(x + r, y + h);
        ctx.quadraticCurveTo(x, y + h, x, y + h - r);
        ctx.lineTo(x, y + r);
        ctx.quadraticCurveTo(x, y, x + r, y);
        ctx.closePath();

        ctx.fillStyle = bgColor;
        ctx.fill();

        ctx.lineWidth = 3;
        ctx.strokeStyle = borderColor;
        ctx.shadowColor = borderColor;
        ctx.shadowBlur = 12;
        ctx.stroke();
        ctx.shadowBlur = 0;

        // Status indicator dot
        ctx.beginPath();
        ctx.arc(28, 32, 6, 0, Math.PI * 2);
        ctx.fillStyle = dotColor;
        ctx.shadowColor = dotColor;
        ctx.shadowBlur = 8;
        ctx.fill();
        ctx.shadowBlur = 0;

        // Drone ID number
        ctx.font = 'bold 22px "JetBrains Mono", Consolas, monospace';
        ctx.fillStyle = '#FFFFFF';
        ctx.textAlign = 'left';
        ctx.textBaseline = 'middle';
        ctx.fillText(labelText, 44, 32);

        // Role subtext
        ctx.font = 'bold 15px "JetBrains Mono", Consolas, monospace';
        ctx.fillStyle = textColor;
        ctx.textAlign = 'right';
        ctx.fillText(subText, 238, 32);

        texture.needsUpdate = true;
    }

    // =========================================================================
    // 4. Telemetry Frame Ingestion & Interpolation
    // =========================================================================
    function onTelemetryFrame(frame) {
        lastTelemetryTime = performance.now();

        // Update Top Bar HUD Info
        updateTopBar(frame);

        // Update Target Waypoint position
        if (frame.target && targetBeacon) {
            targetBeacon.position.set(frame.target[0], frame.target[1], frame.target[2]);
        }

        // Update / Spawn Drones
        const activeIds = new Set();
        let currentLeaderDrone = null;

        if (Array.isArray(frame.drones)) {
            frame.drones.forEach(d => {
                activeIds.add(d.id);
                let entry = dronesMap.get(d.id);

                if (!entry) {
                    const mesh = buildQuadcopterMesh(d.is_leader);
                    scene.add(mesh);
                    const dropLine = createDropLine(d.is_leader);
                    const shadow = createGroundShadow(d.is_leader);
                    const badge = createDroneBadge(d.id, d.is_leader);
                    entry = {
                        mesh: mesh,
                        dropLine: dropLine,
                        shadow: shadow,
                        badge: badge,
                        targetPos: new THREE.Vector3(...d.pos),
                        currentPos: new THREE.Vector3(...d.pos),
                        yaw: d.yaw || 0,
                        pitch: d.pitch || 0,
                        roll: d.roll || 0,
                        is_leader: d.is_leader,
                        crashed: d.crashed,
                        rotorSpeed: d.crashed ? 0 : 0.4,
                    };
                    dronesMap.set(d.id, entry);
                }

                // Update target kinematics for interpolation
                entry.targetPos.set(d.pos[0], d.pos[1], d.pos[2]);
                entry.targetYaw = d.yaw || 0;
                entry.targetPitch = d.pitch || 0;
                entry.targetRoll = d.roll || 0;
                entry.crashed = d.crashed;
                entry.is_leader = d.is_leader;
                entry.status = d.status;
                entry.rotorSpeed = d.crashed ? 0 : 0.45;

                // Update 3D Tactical ID Badge
                updateDroneBadge(entry.badge, d);

                // Dynamic Rotor Disc, Core, Drop-Line & Shadow Role Tinting
                let rotorColor = COLORS.escort;
                let rotorOpacity = 0.45;
                let indicatorColor = COLORS.dropLine;

                if (d.crashed) {
                    rotorColor = 0x661122; // Dim dead red
                    rotorOpacity = 0.2;
                    indicatorColor = COLORS.crashed;
                } else if (d.is_leader) {
                    rotorColor = COLORS.leader; // Bright neon amber
                    rotorOpacity = 0.65;
                    indicatorColor = COLORS.leader;
                } else if (d.status === 'PROPAGATING') {
                    rotorColor = COLORS.propagating; // Pulse echo amber
                    rotorOpacity = 0.55;
                    indicatorColor = COLORS.propagating;
                }

                // Update rotor discs
                if (entry.mesh.userData.rotors) {
                    entry.mesh.userData.rotors.forEach(r => {
                        r.material.color.setHex(rotorColor);
                        r.material.opacity = rotorOpacity;
                    });
                }

                // Update core
                const coreMesh = entry.mesh.getObjectByName('core');
                if (coreMesh) {
                    coreMesh.material.color.setHex(indicatorColor);
                }

                // Update drop line and shadow colors
                if (entry.dropLine && entry.dropLine.material) {
                    entry.dropLine.material.color.setHex(indicatorColor);
                }
                if (entry.shadow && entry.shadow.material) {
                    entry.shadow.material.color.setHex(indicatorColor);
                }

                if (d.is_leader && !d.crashed) {
                    currentLeaderDrone = entry;
                }
            });
        }

        // Clean up vanished drones
        for (let [id, entry] of dronesMap.entries()) {
            if (!activeIds.has(id)) {
                scene.remove(entry.mesh);
                if (entry.dropLine) {
                    scene.remove(entry.dropLine);
                    entry.dropLine.geometry.dispose();
                    entry.dropLine.material.dispose();
                }
                if (entry.shadow) {
                    scene.remove(entry.shadow);
                    entry.shadow.geometry.dispose();
                    entry.shadow.material.dispose();
                }
                if (entry.badge) {
                    scene.remove(entry.badge);
                    if (entry.badge.material.map) entry.badge.material.map.dispose();
                    entry.badge.material.dispose();
                }
                dronesMap.delete(id);
            }
        }

        // Update Dynamic Catenary Tether
        updateTetherCurve(frame.tether, currentLeaderDrone);
    }

    function updateTetherCurve(tetherData, leaderEntry) {
        if (!tetherMesh) return;

        if (!tetherData || tetherData.severed || !leaderEntry) {
            tetherMesh.visible = false;
            return;
        }

        tetherMesh.visible = true;
        const anchor = new THREE.Vector3(0, 0, 1.5);
        const leadPos = leaderEntry.currentPos;

        // Generate 16 catenary curve points with slight gravity sag
        const points = [];
        const segments = 16;
        for (let i = 0; i <= segments; i++) {
            const t = i / segments;
            const p = new THREE.Vector3().lerpVectors(anchor, leadPos, t);
            // Parabolic sag
            const sag = 2.5 * Math.sin(t * Math.PI);
            p.z = Math.max(0.2, p.z - sag);
            points.push(p);
        }

        tetherMesh.geometry.setFromPoints(points);
        tetherMesh.geometry.computeVertexNormals();
    }

    function updateTopBar(frame) {
        const simTimeElem = document.getElementById('hud-sim-time');
        const modeElem = document.getElementById('hud-mode');
        const brainElem = document.getElementById('hud-brain');
        const dronesElem = document.getElementById('hud-drones');

        if (simTimeElem && frame.sim_time !== undefined) {
            simTimeElem.textContent = `${frame.sim_time.toFixed(1)}s`;
        }
        if (modeElem && frame.formation) {
            modeElem.textContent = frame.formation;
        }
        if (brainElem && frame.brain) {
            brainElem.textContent = frame.brain;
        }
        if (dronesElem && Array.isArray(frame.drones)) {
            const alive = frame.drones.filter(d => !d.crashed).length;
            dronesElem.textContent = `${alive}/${frame.drones.length}`;
        }
    }

    function onConnectionStatus(connected) {
        const ind = document.getElementById('hud-conn-ind');
        const text = document.getElementById('hud-conn-text');
        if (ind && text) {
            if (connected) {
                ind.classList.remove('disconnected');
                text.textContent = 'ONLINE (60 FPS)';
            } else {
                ind.classList.add('disconnected');
                text.textContent = 'LINK LOST';
            }
        }
    }

    // =========================================================================
    // 5. Main Animation & Render Loop (60 FPS GPU)
    // =========================================================================
    function animate() {
        requestAnimationFrame(animate);

        // 1. Smooth Drone Position & Rotor Interpolation (lerp)
        for (let [id, entry] of dronesMap.entries()) {
            // Smoothly lerp towards target packet position
            entry.currentPos.lerp(entry.targetPos, 0.35);
            entry.mesh.position.copy(entry.currentPos);

            // Orientation lerp
            entry.mesh.rotation.z = entry.targetYaw;
            entry.mesh.rotation.y = entry.targetPitch;
            entry.mesh.rotation.x = entry.targetRoll;

            // Spin rotors around Z axis
            if (entry.mesh.userData.rotors && !entry.crashed) {
                entry.mesh.userData.rotors.forEach((rotor, idx) => {
                    const dir = (idx % 2 === 0) ? 1 : -1;
                    rotor.rotation.z += entry.rotorSpeed * dir;
                });
            }

            // Update 3D World Drop-Line from (x, y, z) down to (x, y, 0)
            if (entry.dropLine) {
                const p = entry.currentPos;
                if (p.z > 0.3 && !entry.crashed) {
                    entry.dropLine.visible = true;
                    entry.dropLine.geometry.setFromPoints([
                        new THREE.Vector3(p.x, p.y, p.z),
                        new THREE.Vector3(p.x, p.y, 0),
                    ]);
                    entry.dropLine.computeLineDistances();
                } else {
                    entry.dropLine.visible = false;
                }
            }

            // Update Ground Shadow ring: strictly flat on ground plane (z=0.05) directly underneath
            if (entry.shadow) {
                const p = entry.currentPos;
                entry.shadow.position.set(p.x, p.y, 0.05);
                entry.shadow.rotation.set(0, 0, 0);
                entry.shadow.visible = !entry.crashed || p.z > 0.3;
            }

            // Update 3D Tactical ID Badge Position (floating plumb 1.35m above drone)
            if (entry.badge) {
                entry.badge.position.set(entry.currentPos.x, entry.currentPos.y, entry.currentPos.z + 1.35);
            }
        }

        // 2. Animate Target Waypoint Beacon
        if (targetBeacon) {
            const diamond = targetBeacon.getObjectByName('diamond');
            if (diamond) diamond.rotation.z += 0.02;
            const ring = targetBeacon.getObjectByName('ring');
            if (ring) {
                const scale = 1.0 + 0.15 * Math.sin(performance.now() * 0.005);
                ring.scale.set(scale, scale, 1);
                ring.position.z = -targetBeacon.position.z + 0.05;
            }
        }

        // 3. Chase Camera Logic
        if (isChaseCam) {
            let targetEntry = dronesMap.get(targetDroneId);
            if (!targetEntry) {
                // Default to first living drone
                for (let [_, e] of dronesMap) {
                    if (!e.crashed) { targetEntry = e; break; }
                }
            }

            if (targetEntry) {
                const fwdX = Math.cos(targetEntry.targetYaw);
                const fwdY = Math.sin(targetEntry.targetYaw);
                const chaseOffset = new THREE.Vector3(-fwdX * 18, -fwdY * 18, 9);
                const desiredCamPos = targetEntry.currentPos.clone().add(chaseOffset);

                camera.position.lerp(desiredCamPos, 0.08);
                controls.target.lerp(targetEntry.currentPos, 0.1);
            }
        }

        // 4. Update Click Ripples & Orbit Controls & Render
        updateRipples();
        controls.update();
        renderer.render(scene, camera);

        // 5. FPS Counter Calculation
        fpsCounter++;
        const now = performance.now();
        if (now - lastFpsUpdate >= 1000) {
            const fpsElem = document.getElementById('hud-fps');
            if (fpsElem) {
                fpsElem.textContent = `${fpsCounter} FPS`;
            }
            fpsCounter = 0;
            lastFpsUpdate = now;
        }
    }

    // =========================================================================
    // 6. User Interaction, Ground Click Dispatch & Window Controls
    // =========================================================================
    const groundPlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
    const clickRaycaster = new THREE.Raycaster();
    const clickPoint = new THREE.Vector3();
    let pointerDownX = 0, pointerDownY = 0;
    const activeRipples = [];

    function onPointerDown(e) {
        pointerDownX = e.clientX;
        pointerDownY = e.clientY;
    }

    function onPointerUp(e) {
        const dist = Math.hypot(e.clientX - pointerDownX, e.clientY - pointerDownY);
        // Only trigger on clean click without significant drag (< 6px) and left button
        if (dist < 6 && e.button === 0) {
            const chk = document.getElementById('chk-click-target');
            if (chk && !chk.checked) return;

            const rect = renderer.domElement.getBoundingClientRect();
            const mouseX = ((e.clientX - rect.left) / rect.width) * 2 - 1;
            const mouseY = -((e.clientY - rect.top) / rect.height) * 2 + 1;

            clickRaycaster.setFromCamera(new THREE.Vector2(mouseX, mouseY), camera);
            if (clickRaycaster.ray.intersectPlane(groundPlane, clickPoint)) {
                const targetX = Math.round(Math.max(-140, Math.min(140, clickPoint.x)));
                const targetY = Math.round(Math.max(-140, Math.min(140, clickPoint.y)));
                const targetZ = 30.0;

                spawnClickRipple(targetX, targetY);

                if (window.swarmClient) {
                    window.swarmClient.send({ cmd: 'SET_TARGET', pos: [targetX, targetY, targetZ] });
                }
                if (window.tacticalHud) {
                    window.tacticalHud.logEvent('DISPATCH', `Ground click waypoint -> [${targetX}, ${targetY}, ${targetZ}]`);
                }
            }
        }
    }

    function spawnClickRipple(x, y) {
        const geom = new THREE.RingGeometry(0.5, 0.9, 32);
        const mat = new THREE.MeshBasicMaterial({
            color: COLORS.target,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.9,
        });
        const ripple = new THREE.Mesh(geom, mat);
        ripple.position.set(x, y, 0.1);
        scene.add(ripple);

        activeRipples.push({
            mesh: ripple,
            born: performance.now(),
            lifetime: 600,
        });
    }

    function updateRipples() {
        const now = performance.now();
        for (let i = activeRipples.length - 1; i >= 0; i--) {
            const r = activeRipples[i];
            const age = now - r.born;
            if (age >= r.lifetime) {
                scene.remove(r.mesh);
                r.mesh.geometry.dispose();
                r.mesh.material.dispose();
                activeRipples.splice(i, 1);
            } else {
                const progress = age / r.lifetime;
                const scale = 1.0 + progress * 7.0;
                r.mesh.scale.set(scale, scale, 1);
                r.mesh.material.opacity = (1.0 - progress) * 0.9;
            }
        }
    }

    function onWindowResize() {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    }

    function onKeyDown(e) {
        if (e.key === 'c' || e.key === 'C') {
            isChaseCam = !isChaseCam;
            console.log(`[CAM] Chase Cam: ${isChaseCam ? 'ENABLED' : 'DISABLED'}`);
        } else if (e.key === 'r' || e.key === 'R') {
            isChaseCam = false;
            camera.position.set(70, -70, 65);
            controls.target.set(30, 20, 15);
            console.log('[CAM] Camera reset to tactical overview.');
        }
    }

    // Public API for HUD interaction
    window.selectDrone = function(id) {
        targetDroneId = id;
        isChaseCam = true;
        console.log(`[CAM] Focused camera on Drone #${id}`);
    };

    // Initialize when DOM is ready
    window.addEventListener('DOMContentLoaded', init);

})();
