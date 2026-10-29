/**
 * SwarmTech3_UpRising: Cyberpunk Tactical HUD & Command Deck (hud.js)
 * Handles telemetry cards, mission dispatch, formation switching,
 * brain architecture hot-swapping, casualty lab controls, and event logging.
 */

(function () {
    'use strict';

    // State caching
    const droneCards = new Map();
    let currentLeaderId = null;
    let currentFormation = null;
    let currentBrain = null;
    let currentCommsMode = null;
    let isDraggingJammingSlider = false;
    let tetherSevered = false;
    let patrolActive = false;
    let swarmRtbActive = false;
    let overheadMarshalActive = false;
    let recoveryStage = 'IDLE';
    const deadDroneIds = new Set();
    const lostLinkIds = new Set();
    const takeoffDroneIds = new Set();
    const enrouteDroneIds = new Set();
    const jettisonDroneIds = new Set();
    const landedDroneIds = new Set();
    const descendingDroneIds = new Set();

    // DOM Elements
    let cardsContainer = null;
    let eventLogList = null;
    let inputX = null, inputY = null, inputZ = null;

    function init() {
        cardsContainer = document.getElementById('telemetry-cards');
        eventLogList = document.getElementById('event-log-list');
        inputX = document.getElementById('tgt-input-x');
        inputY = document.getElementById('tgt-input-y');
        inputZ = document.getElementById('tgt-input-z');

        setupEventListeners();

        if (window.swarmClient) {
            window.swarmClient.on('telemetry', onTelemetry);
            window.swarmClient.on('command_ack', onCommandAck);
            window.swarmClient.on('status', onWsStatus);
        }

        logEvent('SYSTEM', 'Tactical HUD & Command Deck Initialized.');
    }

    // =========================================================================
    // 1. UI Event Listeners & Command Dispatch
    // =========================================================================
    function setupEventListeners() {
        // Swarm Architecture & Comms Selectors (TKT-17)
        setupBtn('btn-mode-tether', () => dispatchCommsMode('TETHERED_RELAY'));
        setupBtn('btn-mode-star', () => dispatchCommsMode('LEADER_FOLLOWER'));
        setupBtn('btn-mode-mesh', () => dispatchCommsMode('GOSSIP_CONSENSUS'));

        // Electronic Warfare & RF Jamming Slider (PLN-03)
        const jammingSlider = document.getElementById('rf-jamming-slider');
        const jammingVal = document.getElementById('rf-jamming-val');
        const rfEnvStatus = document.getElementById('rf-env-status');
        if (jammingSlider) {
            jammingSlider.addEventListener('input', (e) => {
                isDraggingJammingSlider = true;
                const pct = parseInt(e.target.value);
                const rate = pct / 100.0;
                if (jammingVal) jammingVal.textContent = `${pct}% JAMMING`;
                if (rfEnvStatus) {
                    rfEnvStatus.innerHTML = pct > 0 ? `RF: <strong style="color:var(--neon-amber)">CONTESTED (${pct}%)</strong>` : `RF: <strong>NOMINAL</strong>`;
                }
                window.swarmClient?.send({ cmd: 'SET_RF_JAMMING', rate: rate });
            });
            jammingSlider.addEventListener('change', () => {
                isDraggingJammingSlider = false;
            });
        }

        // Brain Selectors (Legacy / Hot-swap)
        setupBtn('btn-brain-lf', () => dispatchBrain('LEADER_FOLLOWER'));
        setupBtn('btn-brain-gossip', () => dispatchBrain('GOSSIP_CONSENSUS'));

        // Formation Selectors
        setupBtn('btn-form-echelon', () => dispatchFormation('ECHELON_ESCORT'));
        setupBtn('btn-form-boids', () => dispatchFormation('NATURAL_FLOCK'));
        setupBtn('btn-form-vortex', () => dispatchFormation('ORBIT_VORTEX'));
        setupBtn('btn-form-patrol', () => dispatchFormation('PERIMETER_PATROL'));

        // Preset Waypoints
        setupBtn('btn-wp-alpha', () => dispatchTarget([60.0, 40.0, 30.0], 'ALPHA'));
        setupBtn('btn-wp-bravo', () => dispatchTarget([-40.0, 70.0, 45.0], 'BRAVO'));
        setupBtn('btn-wp-charlie', () => dispatchTarget([50.0, -50.0, 25.0], 'CHARLIE'));
        setupBtn('btn-wp-rtb', () => dispatchTarget([0.0, 0.0, 20.0], 'RTB_BASE'));
        setupBtn('btn-toggle-patrol', () => {
            logEvent('CMD', 'Toggling continuous 3D target auto-patrol loop: TOGGLE_PATROL');
            window.swarmClient?.send({ cmd: 'TOGGLE_PATROL' });
        });

        // Custom Target Coordinates
        const btnDispatchCustom = document.getElementById('btn-dispatch-custom');
        if (btnDispatchCustom) {
            btnDispatchCustom.addEventListener('click', () => {
                const x = parseFloat(inputX?.value || 0);
                const y = parseFloat(inputY?.value || 0);
                const z = parseFloat(inputZ?.value || 25);
                dispatchTarget([x, y, z], 'CUSTOM_COORD');
            });
        }

        // 2-Stage Tactical Recovery & Soft-Landing Controls (TKT-19)
        setupBtn('btn-rtb-hold', () => {
            logEvent('CMD', 'Initiating Stage 1: Swarm RTB to overhead sentry hold (0,0,25)');
            window.swarmClient?.send({ cmd: 'RTB_HOLD' });
        });

        setupBtn('btn-land-swarm', () => {
            logEvent('CMD', 'Engaging Stage 2: Swarm ATC landing pad allocation & staggered soft-landing');
            window.swarmClient?.send({ cmd: 'LAND_SWARM' });
        });

        setupBtn('btn-cancel-rtb', () => {
            logEvent('CMD', 'Cancelling recovery: Resuming normal swarm cruise');
            window.swarmClient?.send({ cmd: 'CANCEL_RTB' });
        });

        // Casualty Lab Controls
        setupBtn('btn-kill-leader', () => {
            logEvent('CMD', 'Injecting casualty: KILL_LEADER');
            window.swarmClient?.send({ cmd: 'KILL_LEADER' });
        });

        setupBtn('btn-cut-tether', () => {
            logEvent('CMD', 'Injecting physical casualty: CUT_TETHER');
            window.swarmClient?.send({ cmd: 'CUT_TETHER' });
        });

        setupBtn('btn-kill-random', () => {
            const aliveFollowers = [];
            for (let [id, card] of droneCards) {
                if (!deadDroneIds.has(id) && id !== currentLeaderId) {
                    aliveFollowers.push(id);
                }
            }
            if (aliveFollowers.length > 0) {
                const victimId = aliveFollowers[Math.floor(Math.random() * aliveFollowers.length)];
                logEvent('CMD', `Injecting escort casualty: KILL_DRONE #${victimId}`);
                window.swarmClient?.send({ cmd: 'KILL_DRONE', id: victimId });
            } else {
                logEvent('WARN', 'No living escort followers available to terminate.');
            }
        });

        setupBtn('btn-respawn-dead', () => {
            const deadCount = deadDroneIds.size;
            if (deadCount > 0) {
                logEvent('CMD', `Respawning ${deadCount} destroyed drone(s) in-place: REBOOT_ALL_DEAD`);
                window.swarmClient?.send({ cmd: 'REBOOT_ALL_DEAD' });
            } else {
                logEvent('INFO', 'No destroyed casualties to respawn. Fleet 100% operational.');
            }
        });

        setupBtn('btn-clear-wreckage', () => {
            const deadCount = deadDroneIds.size;
            if (deadCount > 0) {
                logEvent('CMD', `Salvaging ${deadCount} destroyed wreckage: CLEAR_WRECKAGE`);
                window.swarmClient?.send({ cmd: 'CLEAR_WRECKAGE' });
            } else {
                logEvent('INFO', 'No crashed wreckage on battlefield.');
            }
        });

        setupBtn('btn-reinforce-swarm', () => {
            logEvent('CMD', 'Launching reinforcement drone from base pad: LAUNCH_REINFORCEMENT');
            window.swarmClient?.send({ cmd: 'LAUNCH_REINFORCEMENT' });
        });

        setupBtn('btn-reboot-fleet', () => {
            logEvent('CMD', 'Executing tactical swarm reboot: REBOOT_FLEET');
            window.swarmClient?.send({ cmd: 'REBOOT_FLEET' });
        });

        // Panel Toggle Buttons
        const leftToggle = document.getElementById('toggle-left-panel');
        if (leftToggle) {
            leftToggle.addEventListener('click', () => {
                const p = document.getElementById('left-panel');
                p?.classList.toggle('collapsed');
            });
        }

        const rightToggle = document.getElementById('toggle-right-panel');
        if (rightToggle) {
            rightToggle.addEventListener('click', () => {
                const p = document.getElementById('right-panel');
                p?.classList.toggle('collapsed');
            });
        }
    }

    function setupBtn(id, handler) {
        const btn = document.getElementById(id);
        if (btn) {
            btn.addEventListener('click', (e) => {
                e.preventDefault();
                handler();
            });
        }
    }

    function dispatchCommsMode(modeName) {
        logEvent('CMD', `Switching Swarm Architecture -> ${modeName}`);
        window.swarmClient?.send({ cmd: 'SET_COMMS_MODE', mode: modeName });
    }

    function dispatchBrain(brainName) {
        logEvent('CMD', `Hot-swapping Brain Architecture -> ${brainName}`);
        window.swarmClient?.send({ cmd: 'SET_BRAIN', brain: brainName });
    }

    function dispatchFormation(modeName) {
        logEvent('CMD', `Changing Swarm Formation -> ${modeName}`);
        window.swarmClient?.send({ cmd: 'SET_FORMATION', mode: modeName });
    }

    function dispatchTarget(pos, label) {
        patrolActive = false;
        const btnPatrol = document.getElementById('btn-toggle-patrol');
        if (btnPatrol) {
            btnPatrol.classList.remove('active-patrol');
            btnPatrol.innerHTML = '&#x221E; AUTO PATROL LOOP';
        }
        logEvent('CMD', `Dispatching Target [${label}] -> (${pos[0]}, ${pos[1]}, ${pos[2]})`);
        window.swarmClient?.send({ cmd: 'SET_TARGET', pos: pos });
        if (inputX) inputX.value = pos[0];
        if (inputY) inputY.value = pos[1];
        if (inputZ) inputZ.value = pos[2];
    }

    // =========================================================================
    // 2. Telemetry Ingestion & Real-Time Card Updates
    // =========================================================================
    function onTelemetry(frame) {
        if (!frame) return;

        // 0. Update Comms Mode & RF Jamming (TKT-17)
        if (frame.comms_mode && frame.comms_mode !== currentCommsMode) {
            currentCommsMode = frame.comms_mode;
            updateCommsButtons(currentCommsMode);
        }
        if (frame.rf_jamming !== undefined && !isDraggingJammingSlider) {
            const pct = Math.round(frame.rf_jamming * 100);
            const slider = document.getElementById('rf-jamming-slider');
            const val = document.getElementById('rf-jamming-val');
            const rfEnvStatus = document.getElementById('rf-env-status');
            if (slider && parseInt(slider.value) !== pct) slider.value = pct;
            if (val) val.textContent = `${pct}% JAMMING`;
            if (rfEnvStatus) {
                rfEnvStatus.innerHTML = pct > 0 ? `RF: <strong style="color:var(--neon-amber)">CONTESTED (${pct}%)</strong>` : `RF: <strong>NOMINAL</strong>`;
            }
        }

        // 1. Update Brain selection highlighting
        if (frame.brain && frame.brain !== currentBrain) {
            currentBrain = frame.brain;
            updateBrainButtons(currentBrain);
        }

        // 2. Update Formation selection highlighting
        if (frame.formation && frame.formation !== currentFormation) {
            currentFormation = frame.formation;
            updateFormationButtons(currentFormation);
        }

        // 3. Monitor Tether Severance
        if (frame.tether) {
            if (frame.tether.severed && !tetherSevered) {
                tetherSevered = true;
                logEvent('ALERT', 'Physical Tether Severed! Primary power cut.');
            } else if (!frame.tether.severed && tetherSevered) {
                tetherSevered = false;
                logEvent('STATUS', 'Tether re-anchored to GCS mast.');
            }
        }

        // 4. Update Target coordinate inputs if empty
        if (frame.target && inputX && !inputX.matches(':focus')) {
            inputX.value = frame.target[0];
            inputY.value = frame.target[1];
            inputZ.value = frame.target[2];
        }

        // 4b. Monitor Auto-Patrol Loop State
        if (typeof frame.patrol_active === 'boolean' && frame.patrol_active !== patrolActive) {
            patrolActive = frame.patrol_active;
            const btnPatrol = document.getElementById('btn-toggle-patrol');
            if (btnPatrol) {
                if (patrolActive) {
                    btnPatrol.classList.add('active-patrol');
                    btnPatrol.innerHTML = '&#x221E; PATROL ACTIVE [ON]';
                    logEvent('STATUS', '3D Target Auto-Patrol Loop ENGAGED.');
                } else {
                    btnPatrol.classList.remove('active-patrol');
                    btnPatrol.innerHTML = '&#x221E; AUTO PATROL LOOP';
                    logEvent('STATUS', '3D Target Auto-Patrol Loop DISENGAGED.');
                }
            }
        }

        // 4c. Monitor Tactical Recovery & Soft-Landing (TKT-19)
        const marshalActive = !!frame.overhead_marshal_active;
        const stage = frame.recovery_stage || 'IDLE';

        if (marshalActive !== overheadMarshalActive) {
            overheadMarshalActive = marshalActive;
            const btnRtb = document.getElementById('btn-rtb-hold');
            if (btnRtb) {
                btnRtb.classList.toggle('active-recovery', overheadMarshalActive);
            }
            if (overheadMarshalActive) {
                logEvent('RECOVERY', 'Stage 1: Swarm transiting to Base (0, 0, 25m) overhead marshal hold.');
            }
        }

        if (stage !== recoveryStage) {
            recoveryStage = stage;
            const btnLand = document.getElementById('btn-land-swarm');
            if (btnLand) {
                btnLand.classList.toggle('active-recovery', recoveryStage === 'LANDING');
            }
            if (recoveryStage === 'LANDING') {
                logEvent('RECOVERY', 'Stage 2: Coordinated ATC Soft-Landing initiated with 2.0s stagger intervals.');
            } else if (recoveryStage === 'IDLE' && !overheadMarshalActive) {
                logEvent('STATUS', 'Recovery sequence cleared. Fleet in normal cruise.');
            }
        }

        const btnCancelRtb = document.getElementById('btn-cancel-rtb');
        if (btnCancelRtb) {
            if (overheadMarshalActive || recoveryStage === 'LANDING') {
                btnCancelRtb.style.display = 'block';
            } else {
                btnCancelRtb.style.display = 'none';
            }
        }

        // 4d. Update Top Bar Failsafe Indicator (TKT-16, FIX-02)
        const failsafePill = document.getElementById('hud-failsafe-pill');
        const failsafeText = document.getElementById('hud-failsafe-text');
        if (failsafePill && failsafeText) {
            if (frame.tether && frame.tether.severed) {
                failsafePill.classList.remove('hidden');
                if (frame.recovery_stage === 'LANDING' || frame.recovery_stage === 'COMPLETE') {
                    failsafePill.className = 'status-pill failsafe-pill failsafe-rtb';
                    failsafeText.textContent = 'TACTICAL RECOVERY: COORD SOFT-LANDING ACTIVE';
                } else if (frame.swarm_rtb_active) {
                    failsafePill.className = 'status-pill failsafe-pill failsafe-rtb';
                    failsafeText.textContent = 'UPLINK LOST: DOCTRINE A SWARM RTB TO BASE ACTIVE';
                } else if (frame.lost_link_loiter_timer !== undefined && frame.lost_link_loiter_timer > 0) {
                    failsafePill.className = 'status-pill failsafe-pill failsafe-loiter';
                    const leaderTwoIc = frame.two_ic_id !== null && frame.two_ic_id !== undefined ? frame.two_ic_id : 1;
                    failsafeText.textContent = `UPLINK SEVERED: 2IC #${leaderTwoIc} IN COMMAND | DOCTRINE A LOITER: ${frame.lost_link_loiter_timer.toFixed(1)}s -> RTB`;
                } else {
                    failsafePill.className = 'status-pill failsafe-pill failsafe-untethered';
                    failsafeText.textContent = 'UNTETHERED CRUISE (MISSION RESUMED)';
                }
            } else {
                failsafePill.classList.add('hidden');
            }
        }

        // Failsafe Event Logging
        if (frame.tether && frame.tether.severed && !tetherSevered) {
            tetherSevered = true;
            const twoIcStr = frame.two_ic_id !== null && frame.two_ic_id !== undefined ? `#${frame.two_ic_id}` : '#1';
            logEvent('FAILSAFE', `Tactical umbilical severed! 2IC ${twoIcStr} assumed command. Drone #0 breakaway dive initiated.`);
            logEvent('DOCTRINE_A', `Doctrine A engaged: Holding 30s defensive loiter before automated RTB.`);
        } else if (frame.tether && !frame.tether.severed && tetherSevered) {
            tetherSevered = false;
            logEvent('STATUS', `Tactical umbilical reconnected / swarm rebooted.`);
        }

        if (frame.swarm_rtb_active && !swarmRtbActive) {
            swarmRtbActive = true;
            logEvent('RTB', `Doctrine A 30s loiter grace period expired. Swarm RTB initiated to Base (0, 0, 25m).`);
        } else if (!frame.swarm_rtb_active && swarmRtbActive) {
            swarmRtbActive = false;
        }

        // 5. Update Drone Telemetry Cards
        if (Array.isArray(frame.drones) && cardsContainer) {
            let leaderFound = null;

            // Purge card DOM elements and tracking sets for vanished drones (e.g. cleared wreckage)
            const activeIds = new Set(frame.drones.map(d => d.id));
            for (let [id, card] of droneCards.entries()) {
                if (!activeIds.has(id)) {
                    card.el.remove();
                    droneCards.delete(id);
                    deadDroneIds.delete(id);
                    lostLinkIds.delete(id);
                    takeoffDroneIds.delete(id);
                    enrouteDroneIds.delete(id);
                    landedDroneIds.delete(id);
                    descendingDroneIds.delete(id);
                }
            }

            frame.drones.forEach(d => {
                if (d.is_leader && !d.crashed) {
                    leaderFound = d.id;
                }

                // Detect Autonomic Events & State Changes
                if (d.crashed && !deadDroneIds.has(d.id)) {
                    deadDroneIds.add(d.id);
                    logEvent('CASUALTY', `Drone #${d.id} crashed! Ballistic drop initiated.`);
                } else if (!d.crashed && deadDroneIds.has(d.id)) {
                    deadDroneIds.delete(d.id);
                    logEvent('STATUS', `Drone #${d.id} rebooted and active.`);
                }

                if (d.flight_state === 'TAKEOFF' && !takeoffDroneIds.has(d.id)) {
                    takeoffDroneIds.add(d.id);
                    logEvent('LAUNCH', `Reinforcement Drone #${d.id} launched from base pad.`);
                } else if (d.flight_state !== 'TAKEOFF' && takeoffDroneIds.has(d.id)) {
                    takeoffDroneIds.delete(d.id);
                    if (d.flight_state === 'EN_ROUTE') {
                        logEvent('STATUS', `Drone #${d.id} reached transition altitude (20m). En route to formation.`);
                    }
                }

                if (d.flight_state === 'EN_ROUTE' && !enrouteDroneIds.has(d.id)) {
                    enrouteDroneIds.add(d.id);
                } else if (d.flight_state === 'CRUISE' && enrouteDroneIds.has(d.id)) {
                    enrouteDroneIds.delete(d.id);
                    logEvent('STATUS', `Drone #${d.id} reached formation slot. Status: CRUISE.`);
                }

                if (d.flight_state === 'DESCENDING' && !descendingDroneIds.has(d.id)) {
                    descendingDroneIds.add(d.id);
                    const padStr = d.landing_pad_idx !== undefined && d.landing_pad_idx !== null ? `Pad #${d.landing_pad_idx}` : 'designated pad';
                    logEvent('ATC', `Drone #${d.id} allocated ${padStr}. Commencing 2-stage deceleration descent.`);
                } else if (d.flight_state !== 'DESCENDING' && descendingDroneIds.has(d.id)) {
                    descendingDroneIds.delete(d.id);
                }

                if (d.flight_state === 'LANDED' && !landedDroneIds.has(d.id)) {
                    landedDroneIds.add(d.id);
                    const padStr = d.landing_pad_idx !== undefined && d.landing_pad_idx !== null ? `Pad #${d.landing_pad_idx}` : 'pad';
                    logEvent('TOUCHDOWN', `Drone #${d.id} touched down on ${padStr}. Motors disarmed (0 RPM).`);
                } else if (d.flight_state !== 'LANDED' && landedDroneIds.has(d.id)) {
                    landedDroneIds.delete(d.id);
                    logEvent('TAKEOFF', `Drone #${d.id} launching from pad. Rejoining swarm.`);
                }

                if (d.lost_link && !lostLinkIds.has(d.id)) {
                    lostLinkIds.add(d.id);
                    logEvent('WARN', `Drone #${d.id} LOST LINK! Seeking LKP (${d.lkp[0]}, ${d.lkp[1]}, ${d.lkp[2]}).`);
                } else if (!d.lost_link && lostLinkIds.has(d.id)) {
                    lostLinkIds.delete(d.id);
                    logEvent('STATUS', `Drone #${d.id} link restored. Rejoined swarm.`);
                }

                if (d.status === 'BREAKAWAY_JETTISON' && !jettisonDroneIds.has(d.id)) {
                    jettisonDroneIds.add(d.id);
                    logEvent('FAILSAFE', `Drone #${d.id} executing 3D breakaway dive (ΔZ=-4m) and cable purge.`);
                } else if (d.status !== 'BREAKAWAY_JETTISON' && jettisonDroneIds.has(d.id)) {
                    jettisonDroneIds.delete(d.id);
                    logEvent('STATUS', `Drone #${d.id} umbilical purge complete. Rejoining formation as unencumbered escort.`);
                }

                // Update / Build Card DOM
                updateOrCreateDroneCard(d);
            });

            // Detect Autonomic Succession
            if (leaderFound !== null && currentLeaderId !== null && leaderFound !== currentLeaderId) {
                logEvent('SUCCESSION', `Autonomic Succession: Drone #${leaderFound} promoted to Swarm Leader!`);
            }
            currentLeaderId = leaderFound;
        }
    }

    function updateOrCreateDroneCard(drone) {
        let card = droneCards.get(drone.id);

        if (!card) {
            card = createCardElement(drone);
            cardsContainer.appendChild(card.el);
            droneCards.set(drone.id, card);
        }

        // Update In-Place for High-FPS smoothness
        const speed = Math.hypot(drone.vel[0], drone.vel[1], drone.vel[2]).toFixed(1);
        const alt = drone.pos[2].toFixed(1);

        card.elAlt.textContent = `${alt}m`;
        card.elVel.textContent = `${speed}m/s`;
        card.elBat.textContent = `${drone.battery.toFixed(0)}%`;
        card.elBatBar.style.width = `${Math.max(0, Math.min(100, drone.battery))}%`;

        // Signal RSSI & Link Quality (TKT-17)
        if (card.elRssi) {
            const rssi = drone.rssi !== undefined ? drone.rssi.toFixed(0) : '-45';
            const lq = drone.link_quality !== undefined ? drone.link_quality : 1.0;
            card.elRssi.textContent = `${rssi}dBm`;
            if (lq < 0.35) {
                card.elRssi.style.color = 'var(--neon-red)';
            } else if (lq < 0.7) {
                card.elRssi.style.color = 'var(--neon-amber)';
            } else {
                card.elRssi.style.color = 'var(--neon-cyan)';
            }
            if (card.elSignalBar) {
                const sigPct = Math.max(0, Math.min(100, Math.round(lq * 100)));
                card.elSignalBar.style.width = `${sigPct}%`;
                if (lq < 0.35) {
                    card.elSignalBar.style.background = 'var(--neon-red)';
                } else if (lq < 0.7) {
                    card.elSignalBar.style.background = 'var(--neon-amber)';
                } else {
                    card.elSignalBar.style.background = 'linear-gradient(90deg, var(--neon-amber), var(--neon-cyan))';
                }
            }
        }

        // Status Styling & Badges
        card.el.classList.remove('leader-card', 'crashed-card', 'lkp-card', 'gossip-card', 'takeoff-card', 'enroute-card', 'jettison-card', 'landed-card', 'landing-card', 'hold-card');
        card.elRole.className = 'drone-role-badge';

        if (drone.crashed) {
            card.el.classList.add('crashed-card');
            card.elStatus.textContent = 'CRASHED';
            card.elStatus.className = 'status-tag tag-crashed';
            card.elRole.textContent = 'DESTROYED';
            card.elActionBtn.textContent = 'REBOOT';
            card.elActionBtn.className = 'card-btn-action btn-reboot';
            if (card.elLandBtn) card.elLandBtn.style.display = 'none';
        } else if (drone.flight_state === 'LANDED') {
            card.el.classList.add('landed-card');
            const padStr = drone.landing_pad_idx !== undefined && drone.landing_pad_idx !== null ? `P${drone.landing_pad_idx}` : '';
            card.elStatus.textContent = padStr ? `LANDED [${padStr}]` : 'LANDED';
            card.elStatus.className = 'status-tag tag-landed';
            card.elRole.textContent = 'DISARMED';
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'LAUNCH';
                card.elLandBtn.className = 'card-btn-action btn-launch';
            }
            card.elActionBtn.textContent = 'KILL';
            card.elActionBtn.className = 'card-btn-action btn-kill';
        } else if (drone.flight_state === 'DESCENDING') {
            card.el.classList.add('landing-card');
            const padStr = drone.landing_pad_idx !== undefined && drone.landing_pad_idx !== null ? `P${drone.landing_pad_idx}` : '';
            card.elStatus.textContent = padStr ? `LANDING [${padStr}]` : 'LANDING';
            card.elStatus.className = 'status-tag tag-landing';
            card.elRole.textContent = 'ATC DESCENT';
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'LANDING';
                card.elLandBtn.className = 'card-btn-action btn-landing-busy';
            }
            card.elActionBtn.textContent = 'KILL';
            card.elActionBtn.className = 'card-btn-action btn-kill';
        } else if (drone.status === 'BREAKAWAY_JETTISON') {
            card.el.classList.add('jettison-card');
            card.elStatus.textContent = 'JETTISONING';
            card.elStatus.className = 'status-tag tag-jettison';
            card.elRole.textContent = '⏏ JETTISON';
            card.elRole.classList.add('badge-jettison');
            if (card.elLandBtn) card.elLandBtn.style.display = 'none';
            card.elActionBtn.textContent = 'KILL';
            card.elActionBtn.className = 'card-btn-action btn-kill';
        } else if (drone.is_leader) {
            card.el.classList.add('leader-card');
            card.elStatus.textContent = overheadMarshalActive ? 'OVERHEAD HOLD' : 'LEADER';
            card.elStatus.className = overheadMarshalActive ? 'status-tag tag-hold' : 'status-tag tag-leader';
            card.elRole.textContent = 'FLAGSHIP';
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'LAND';
                card.elLandBtn.className = 'card-btn-action btn-land';
            }
            card.elActionBtn.textContent = 'KILL';
            card.elActionBtn.className = 'card-btn-action btn-kill';
        } else if (drone.is_2ic) {
            card.elStatus.textContent = overheadMarshalActive ? 'OVERHEAD HOLD' : 'ACTIVE';
            card.elStatus.className = overheadMarshalActive ? 'status-tag tag-hold' : 'status-tag tag-active';
            card.elRole.textContent = '2IC';
            card.elRole.classList.add('badge-2ic');
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'LAND';
                card.elLandBtn.className = 'card-btn-action btn-land';
            }
            card.elActionBtn.textContent = 'KILL';
            card.elActionBtn.className = 'card-btn-action btn-kill';
        } else if (drone.flight_state === 'TAKEOFF') {
            card.el.classList.add('takeoff-card');
            card.elStatus.textContent = 'TAKEOFF';
            card.elStatus.className = 'status-tag tag-takeoff';
            card.elRole.textContent = 'REINFORCE';
            if (card.elLandBtn) card.elLandBtn.style.display = 'none';
            card.elActionBtn.textContent = 'KILL';
            card.elActionBtn.className = 'card-btn-action btn-kill';
        } else if (drone.flight_state === 'EN_ROUTE') {
            card.el.classList.add('enroute-card');
            card.elStatus.textContent = 'EN ROUTE';
            card.elStatus.className = 'status-tag tag-enroute';
            card.elRole.textContent = `TIER-${drone.tier}`;
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'LAND';
                card.elLandBtn.className = 'card-btn-action btn-land';
            }
            card.elActionBtn.textContent = 'KILL';
            card.elActionBtn.className = 'card-btn-action btn-kill';
        } else if (drone.lost_link) {
            card.el.classList.add('lkp-card');
            card.elStatus.textContent = 'LKP RECOVERY';
            card.elStatus.className = 'status-tag tag-lkp';
            card.elRole.textContent = `TIER-${drone.tier}`;
            if (card.elLandBtn) card.elLandBtn.style.display = 'none';
            card.elActionBtn.textContent = 'KILL';
            card.elActionBtn.className = 'card-btn-action btn-kill';
        } else if (drone.status === 'PROPAGATING') {
            card.el.classList.add('gossip-card');
            card.elStatus.textContent = 'ECHO WAVE';
            card.elStatus.className = 'status-tag tag-gossip';
            card.elRole.textContent = `TIER-${drone.tier}`;
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'LAND';
                card.elLandBtn.className = 'card-btn-action btn-land';
            }
            card.elActionBtn.textContent = 'KILL';
            card.elActionBtn.className = 'card-btn-action btn-kill';
        } else {
            card.elStatus.textContent = overheadMarshalActive ? 'OVERHEAD HOLD' : 'ACTIVE';
            card.elStatus.className = overheadMarshalActive ? 'status-tag tag-hold' : 'status-tag tag-active';
            card.elRole.textContent = `TIER-${drone.tier}`;
            if (card.elLandBtn) {
                card.elLandBtn.style.display = 'inline-block';
                card.elLandBtn.textContent = 'LAND';
                card.elLandBtn.className = 'card-btn-action btn-land';
            }
            card.elActionBtn.textContent = 'KILL';
            card.elActionBtn.className = 'card-btn-action btn-kill';
        }
    }

    function createCardElement(drone) {
        const el = document.createElement('div');
        el.className = 'drone-telemetry-card';
        el.id = `card-drone-${drone.id}`;

        el.innerHTML = `
            <div class="card-header">
                <div class="card-id-group">
                    <span class="drone-dot"></span>
                    <strong class="drone-id-label">DRONE #${drone.id}</strong>
                    <span class="drone-role-badge">TIER-${drone.tier}</span>
                </div>
                <span class="status-tag tag-active">ACTIVE</span>
            </div>
            <div class="card-metrics">
                <div class="metric-col">
                    <span class="metric-label">ALT</span>
                    <strong class="metric-val metric-alt">0.0m</strong>
                </div>
                <div class="metric-col">
                    <span class="metric-label">SPEED</span>
                    <strong class="metric-val metric-vel">0.0m/s</strong>
                </div>
                <div class="metric-col">
                    <span class="metric-label">BATTERY</span>
                    <strong class="metric-val metric-bat">100%</strong>
                </div>
                <div class="metric-col">
                    <span class="metric-label">SIGNAL</span>
                    <strong class="metric-val metric-rssi">-45dBm</strong>
                </div>
            </div>
            <div class="card-bars-container">
                <div class="card-battery-progress" title="Battery Level">
                    <div class="card-battery-fill" style="width: 100%;"></div>
                </div>
                <div class="card-signal-progress" title="RF Link Quality">
                    <div class="card-signal-fill" style="width: 100%;"></div>
                </div>
            </div>
            <div class="card-footer">
                <button class="card-btn-action btn-land-action btn-land">LAND</button>
                <button class="card-btn-action btn-kill">KILL</button>
            </div>
        `;

        const actionBtn = el.querySelector('.btn-kill');
        const landBtn = el.querySelector('.btn-land-action');

        actionBtn.addEventListener('click', (e) => {
            e.stopPropagation();
            if (deadDroneIds.has(drone.id)) {
                logEvent('CMD', `Rebooting drone #${drone.id}`);
                window.swarmClient?.send({ cmd: 'REBOOT_DRONE', id: drone.id });
            } else {
                logEvent('CMD', `Terminating drone #${drone.id}`);
                window.swarmClient?.send({ cmd: 'KILL_DRONE', id: drone.id });
            }
        });

        landBtn.addEventListener('click', (e) => {
            e.stopPropagation();
            if (landedDroneIds.has(drone.id)) {
                logEvent('CMD', `Launching Drone #${drone.id} from pad: TAKEOFF_DRONE`);
                window.swarmClient?.send({ cmd: 'TAKEOFF_DRONE', id: drone.id });
            } else {
                logEvent('CMD', `Commanding Drone #${drone.id} to soft-land: LAND_DRONE`);
                window.swarmClient?.send({ cmd: 'LAND_DRONE', id: drone.id });
            }
        });

        el.addEventListener('click', () => {
            logEvent('NAV', `Tactical camera focused on Drone #${drone.id}`);
            if (window.selectDrone) {
                window.selectDrone(drone.id);
            }
        });

        return {
            el: el,
            elAlt: el.querySelector('.metric-alt'),
            elVel: el.querySelector('.metric-vel'),
            elBat: el.querySelector('.metric-bat'),
            elBatBar: el.querySelector('.card-battery-fill'),
            elRssi: el.querySelector('.metric-rssi'),
            elSignalBar: el.querySelector('.card-signal-fill'),
            elStatus: el.querySelector('.status-tag'),
            elRole: el.querySelector('.drone-role-badge'),
            elLandBtn: landBtn,
            elActionBtn: actionBtn,
        };
    }

    // =========================================================================
    // 3. Selection Highlighting
    // =========================================================================
    function updateCommsButtons(activeMode) {
        const tetherBtn = document.getElementById('btn-mode-tether');
        const starBtn = document.getElementById('btn-mode-star');
        const meshBtn = document.getElementById('btn-mode-mesh');

        [tetherBtn, starBtn, meshBtn].forEach(b => b?.classList.remove('active-selection'));

        if (activeMode === 'TETHERED_RELAY') {
            tetherBtn?.classList.add('active-selection');
        } else if (activeMode === 'LEADER_FOLLOWER') {
            starBtn?.classList.add('active-selection');
        } else if (activeMode === 'GOSSIP_CONSENSUS') {
            meshBtn?.classList.add('active-selection');
        }
    }

    function updateBrainButtons(activeBrain) {
        const lfBtn = document.getElementById('btn-brain-lf');
        const gossipBtn = document.getElementById('btn-brain-gossip');

        if (activeBrain === 'LEADER_FOLLOWER') {
            lfBtn?.classList.add('active-selection');
            gossipBtn?.classList.remove('active-selection');
        } else if (activeBrain === 'GOSSIP_CONSENSUS') {
            gossipBtn?.classList.add('active-selection');
            lfBtn?.classList.remove('active-selection');
        }
    }

    function updateFormationButtons(activeFormation) {
        const map = {
            'ECHELON_ESCORT': 'btn-form-echelon',
            'NATURAL_FLOCK': 'btn-form-boids',
            'ORBIT_VORTEX': 'btn-form-vortex',
            'PERIMETER_PATROL': 'btn-form-patrol',
        };

        for (let [mode, btnId] of Object.entries(map)) {
            const btn = document.getElementById(btnId);
            if (btn) {
                if (mode === activeFormation) {
                    btn.classList.add('active-selection');
                } else {
                    btn.classList.remove('active-selection');
                }
            }
        }
    }

    // =========================================================================
    // 4. Tactical Event Logging Feed
    // =========================================================================
    function logEvent(type, message) {
        if (!eventLogList) return;

        const timeStr = new Date().toTimeString().split(' ')[0];
        const item = document.createElement('li');
        item.className = `event-item type-${type.toLowerCase()}`;
        item.innerHTML = `<span class="event-time">[${timeStr}]</span> <span class="event-type">[${type}]</span> <span class="event-msg">${message}</span>`;

        eventLogList.appendChild(item);

        // Keep maximum 30 log lines
        while (eventLogList.children.length > 30) {
            eventLogList.removeChild(eventLogList.firstChild);
        }

        // Auto-scroll to latest
        eventLogList.scrollTop = eventLogList.scrollHeight;
    }

    function onCommandAck(ack) {
        if (ack && ack.cmd) {
            if (ack.cmd === 'CLEAR_WRECKAGE' && Array.isArray(ack.purged_ids)) {
                logEvent('ACK', `Salvage complete: Cleared ${ack.count} wreckage [${ack.purged_ids.join(', ')}]. Remaining: ${ack.remaining_drones}.`);
            } else {
                logEvent('ACK', `Command ${ack.cmd} acknowledged: ${JSON.stringify(ack)}`);
            }
        }
    }

    function onWsStatus(connected) {
        if (connected) {
            logEvent('NET', 'Duplex WebSocket link operational (60 FPS telemetry stream).');
        } else {
            logEvent('NET', 'WebSocket disconnected. Entering reconnection loop...');
        }
    }

    // Export logger for global access (e.g. from app.js)
    window.tacticalHud = {
        logEvent: logEvent,
    };

    window.addEventListener('DOMContentLoaded', init);
})();
