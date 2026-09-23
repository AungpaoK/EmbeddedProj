import React, { useState, useEffect } from 'react';
import {
  Wifi,
  Volume2,
  VolumeX,
  Maximize,
  ArrowUpToLine,
  ArrowDownToLine,
  CheckCircle,
  Timer,
  RotateCcw,
  MapPin,
  X,
  Settings,
  Battery,
  Bluetooth,
  ChevronRight,
  Navigation,
  Home,
} from 'lucide-react';
import './App.css';

// ─── Robot Eyes ───
type EyeMood = 'navigating' | 'arrived' | 'thankyou' | 'returning';

function RobotEyes({ mood }: { mood: EyeMood }) {
  return (
    <div className={`robot-eyes-wrap robot-eyes-wrap--${mood}`}>
      <svg className="robot-eyes-svg" viewBox="0 0 200 100" xmlns="http://www.w3.org/2000/svg">
        {/* Left Eye */}
        <g className="eye-left">
          {mood === 'thankyou' ? (
            <>
              <ellipse cx="52" cy="50" rx="34" ry="34" fill="#fff" stroke="#e2e8f0" strokeWidth="2"/>
              <text x="52" y="62" textAnchor="middle" fontSize="36" fill="#f43f5e" className="eye-heart">♥</text>
            </>
          ) : mood === 'returning' ? (
            <>
              <ellipse cx="52" cy="54" rx="34" ry="28" fill="#fff" stroke="#e2e8f0" strokeWidth="2"/>
              <ellipse cx="52" cy="36" rx="36" ry="20" fill="#c7d7f0"/>
              <ellipse cx="52" cy="60" rx="14" ry="10" fill="#1e3a8a" className="eye-pupil-sleepy"/>
              <ellipse cx="46" cy="56" rx="4" ry="4" fill="#fff" opacity="0.7"/>
            </>
          ) : mood === 'arrived' ? (
            <>
              <ellipse cx="52" cy="50" rx="34" ry="36" fill="#fff" stroke="#e2e8f0" strokeWidth="2"/>
              <ellipse cx="52" cy="53" rx="19" ry="22" fill="#1d4ed8" className="eye-pupil"/>
              <ellipse cx="52" cy="53" rx="9" ry="10" fill="#111827"/>
              <ellipse cx="44" cy="44" rx="6" ry="6" fill="#fff" opacity="0.9"/>
              <ellipse cx="58" cy="48" rx="3" ry="3" fill="#fff" opacity="0.7"/>
            </>
          ) : (
            <>
              <ellipse cx="52" cy="50" rx="34" ry="32" fill="#fff" stroke="#e2e8f0" strokeWidth="2"/>
              <ellipse cx="52" cy="50" rx="16" ry="18" fill="#1d4ed8" className="eye-scan"/>
              <ellipse cx="52" cy="50" rx="7" ry="8" fill="#111827"/>
              <ellipse cx="44" cy="42" rx="5" ry="5" fill="#fff" opacity="0.9"/>
            </>
          )}
        </g>

        {/* Right Eye */}
        <g className="eye-right">
          {mood === 'thankyou' ? (
            <>
              <ellipse cx="148" cy="50" rx="34" ry="34" fill="#fff" stroke="#e2e8f0" strokeWidth="2"/>
              <text x="148" y="62" textAnchor="middle" fontSize="36" fill="#f43f5e" className="eye-heart">♥</text>
            </>
          ) : mood === 'returning' ? (
            <>
              <ellipse cx="148" cy="54" rx="34" ry="28" fill="#fff" stroke="#e2e8f0" strokeWidth="2"/>
              <ellipse cx="148" cy="36" rx="36" ry="20" fill="#c7d7f0"/>
              <ellipse cx="148" cy="60" rx="14" ry="10" fill="#1e3a8a" className="eye-pupil-sleepy"/>
              <ellipse cx="142" cy="56" rx="4" ry="4" fill="#fff" opacity="0.7"/>
            </>
          ) : mood === 'arrived' ? (
            <>
              <ellipse cx="148" cy="50" rx="34" ry="36" fill="#fff" stroke="#e2e8f0" strokeWidth="2"/>
              <ellipse cx="148" cy="53" rx="19" ry="22" fill="#1d4ed8" className="eye-pupil"/>
              <ellipse cx="148" cy="53" rx="9" ry="10" fill="#111827"/>
              <ellipse cx="140" cy="44" rx="6" ry="6" fill="#fff" opacity="0.9"/>
              <ellipse cx="154" cy="48" rx="3" ry="3" fill="#fff" opacity="0.7"/>
            </>
          ) : (
            <>
              <ellipse cx="148" cy="50" rx="34" ry="32" fill="#fff" stroke="#e2e8f0" strokeWidth="2"/>
              <ellipse cx="148" cy="50" rx="16" ry="18" fill="#1d4ed8" className="eye-scan eye-scan-delay"/>
              <ellipse cx="148" cy="50" rx="7" ry="8" fill="#111827"/>
              <ellipse cx="140" cy="42" rx="5" ry="5" fill="#fff" opacity="0.9"/>
            </>
          )}
        </g>

        {/* Blush cheeks */}
        {(mood === 'arrived' || mood === 'thankyou') && (
          <>
            <ellipse cx="22" cy="78" rx="14" ry="8" fill="#fda4af" opacity="0.5"/>
            <ellipse cx="178" cy="78" rx="14" ry="8" fill="#fda4af" opacity="0.5"/>
          </>
        )}
        {/* Zzz for returning */}
        {mood === 'returning' && (
          <>
            <text x="172" y="28" fontSize="14" fill="#93c5fd" fontWeight="800" className="zzz-1">z</text>
            <text x="182" y="16" fontSize="18" fill="#93c5fd" fontWeight="800" className="zzz-2">z</text>
            <text x="192" y="4"  fontSize="22" fill="#93c5fd" fontWeight="800" className="zzz-3">Z</text>
          </>
        )}
      </svg>
    </div>
  );
}

type FSMState = 'KITCHEN_CONFIG' | 'NAVIGATING' | 'ARRIVED' | 'THANK_YOU' | 'RETURNING';

// Which table is assigned to each tray (null = empty)
type TrayAssignment = { tray1: number | null; tray2: number | null; };

const TABLES = [1, 2];

export default function RobotTouchscreenUI() {
  const [fsmState, setFsmState] = useState<FSMState>('KITCHEN_CONFIG');
  const [assignment, setAssignment] = useState<TrayAssignment>({ tray1: null, tray2: null});
  const [soundEnabled, setSoundEnabled] = useState(true);
  const [navProgress, setNavProgress] = useState(15);
  const [thankYouSeconds, setThankYouSeconds] = useState(4);
  const [timeString, setTimeString] = useState('12:00');

  // ordered list of tables to visit (built once at Start press)
  const [queue, setQueue] = useState<number[]>([]);
  // which index in queue we're currently at
  const [queueIndex, setQueueIndex] = useState(0);

  // derived: table we're currently serving
  const currentTable = queue[queueIndex] ?? null;
  // remaining after current
  const remainingTables = queue.slice(queueIndex + 1);

  // ─── Clock ───
  useEffect(() => {
    const update = () => {
      const now = new Date();
      setTimeString(`${String(now.getHours()).padStart(2,'0')}:${String(now.getMinutes()).padStart(2,'0')}`);
    };
    update();
    const id = setInterval(update, 10000);
    return () => clearInterval(id);
  }, []);

  // ─── Navigation progress — reset on each NAVIGATING entry ───
  useEffect(() => {
    let id: ReturnType<typeof setInterval>;
    if (fsmState === 'NAVIGATING') {
      setNavProgress(15);
      id = setInterval(() => setNavProgress(p => p >= 95 ? 95 : p + 5), 350);
    }
    return () => clearInterval(id);
  }, [fsmState, queueIndex]); // re-run when we advance to next table

  // ─── Thank-you countdown → decide next state ───
  useEffect(() => {
    let id: ReturnType<typeof setInterval>;
    if (fsmState === 'THANK_YOU') {
      setThankYouSeconds(4);
      id = setInterval(() => {
        setThankYouSeconds(p => {
          if (p <= 1) {
            clearInterval(id);
            // Advance to next stop
            setQueueIndex(prev => {
              const next = prev + 1;
              if (next < queue.length) {
                // More tables → navigate to next
                setFsmState('NAVIGATING');
              } else {
                // All done → return to kitchen
                setFsmState('RETURNING');
              }
              return next;
            });
            return 0;
          }
          return p - 1;
        });
      }, 1000);
    }
    return () => clearInterval(id);
  }, [fsmState, queue]);

  // ─── Returning auto-reset ───
  useEffect(() => {
    let id: ReturnType<typeof setTimeout>;
    if (fsmState === 'RETURNING') {
      id = setTimeout(() => {
        setAssignment({ tray1: null, tray2: null});
        setQueue([]);
        setQueueIndex(0);
        setFsmState('KITCHEN_CONFIG');
      }, 3500);
    }
    return () => clearTimeout(id);
  }, [fsmState]);

  // ─── Toggle tray assignment (tap again to deselect) ───
  const assignTable = (table: number) => {
    setAssignment(prev => {
      // Already assigned → remove it
      if (prev.tray1 === table) return { ...prev, tray1: null };
      if (prev.tray2 === table) return { ...prev, tray2: null };
      // Not assigned → fill next empty slot
      if (prev.tray1 === null) return { ...prev, tray1: table };
      if (prev.tray2 === null) return { ...prev, tray2: table };
      return prev; // all full
    });
  };

  const removeTray = (tray: keyof TrayAssignment) => {
    setAssignment(prev => ({ ...prev, [tray]: null }));
  };

  // Build ordered queue from tray assignment order (tray1 → tray2)
  const assignedTables = (['tray1','tray2'] as const)
    .map(k => assignment[k])
    .filter((v): v is number => v !== null);

  const canStart = assignedTables.length > 0;

  const handleStart = () => {
    if (!canStart) return;
    setQueue(assignedTables);
    setQueueIndex(0);
    setFsmState('NAVIGATING');
  };

  const trayEntries = [
    { key: 'tray1' as const, label: 'ชั้น 1 (TOP)' },
    { key: 'tray2' as const, label: 'ชั้น 2 (MID)' },
  ];

  // Which tray holds the current table being served
  const currentTrayKey = trayEntries.find(t => assignment[t.key] === currentTable)?.key ?? null;

  return (
    <div className="root-wrapper">
      <div className="kiosk-frame">

        {/* ── Top Status Bar ── */}
        <div className="topbar">
          <div className="topbar-left">
            <span className={`status-pill ${
              fsmState === 'KITCHEN_CONFIG' ? 'pill-idle'
              : fsmState === 'NAVIGATING' ? 'pill-nav'
              : fsmState === 'ARRIVED' ? 'pill-arrived'
              : 'pill-other'
            }`}>
              <span className="pill-dot" />
              {fsmState === 'KITCHEN_CONFIG' && 'IDLE — เลือกโต๊ะเพื่อส่งอาหาร'}
              {fsmState === 'NAVIGATING' && `กำลังเดินทาง → โต๊ะ ${currentTable}`}
              {fsmState === 'ARRIVED' && `ถึงโต๊ะ ${currentTable} แล้ว!`}
              {fsmState === 'THANK_YOU' && `ขอบคุณครับ! โต๊ะ ${currentTable}`}
              {fsmState === 'RETURNING' && 'กำลังกลับครัว...'}
            </span>

            {/* Queue indicator (shown during delivery) */}
            {(fsmState === 'NAVIGATING' || fsmState === 'ARRIVED' || fsmState === 'THANK_YOU') && (
              <div className="queue-indicator">
                {queue.map((t, i) => (
                  <span
                    key={t}
                    className={`queue-stop ${
                      i < queueIndex ? 'queue-stop--done'
                      : i === queueIndex ? 'queue-stop--current'
                      : 'queue-stop--pending'
                    }`}
                  >
                    {i < queueIndex ? '✓' : `T${t}`}
                  </span>
                ))}
                {remainingTables.length > 0 && (
                  <span className="queue-next-hint">ต่อไป → โต๊ะ {remainingTables[0]}</span>
                )}
              </div>
            )}
          </div>

          <div className="topbar-right">
            <Bluetooth className="tb-icon" />
            <Wifi className="tb-icon wifi-on" />
            <Battery className="tb-icon" />
            <span className="tb-time">{timeString}</span>
            <button onClick={() => setSoundEnabled(p => !p)} className="tb-btn">
              {soundEnabled ? <Volume2 className="tb-icon" /> : <VolumeX className="tb-icon muted" />}
            </button>
            <button onClick={() => {
              if (!document.fullscreenElement) document.documentElement.requestFullscreen().catch(()=>{});
              else document.exitFullscreen();
            }} className="tb-btn">
              <Maximize className="tb-icon" />
            </button>
          </div>
        </div>

        {/* ── Main ── */}
        <main className="main-area">

          {/* ══════ KITCHEN CONFIG ══════ */}
          {fsmState === 'KITCHEN_CONFIG' && (
            <div className="config-layout">

              {/* LEFT — Robot Illustration */}
              <div className="robot-panel">
                <div className="robot-illus">
                  <svg className="robot-svg" viewBox="0 0 180 340" fill="none" xmlns="http://www.w3.org/2000/svg">
                    <rect x="20" y="10" width="140" height="280" rx="18" fill="#e8f0fe" stroke="#94a3b8" strokeWidth="2"/>
                    <rect x="30" y="20" width="120" height="40" rx="10" fill="#bfdbfe" stroke="#3b82f6" strokeWidth="1.5"/>
                    <rect x="40" y="26" width="100" height="28" rx="6" fill="#1d4ed8" opacity="0.15"/>
                    <rect x="28" y="72" width="124" height="52" rx="8" fill="#fff" stroke="#e2e8f0" strokeWidth="1.5"/>
                    <rect x="28" y="136" width="124" height="52" rx="8" fill="#fff" stroke="#e2e8f0" strokeWidth="1.5"/>
                    <rect x="28" y="200" width="124" height="52" rx="8" fill="#eff6ff" stroke="#bfdbfe" strokeWidth="1.5"/>
                    <rect x="30" y="262" width="120" height="22" rx="8" fill="#cbd5e1"/>
                    <circle cx="55" cy="300" r="16" fill="#334155"/>
                    <circle cx="55" cy="300" r="9" fill="#64748b"/>
                    <circle cx="125" cy="300" r="16" fill="#334155"/>
                    <circle cx="125" cy="300" r="9" fill="#64748b"/>
                    <path d="M 60 10 Q 90 2 120 10" stroke="#94a3b8" strokeWidth="3" fill="none" strokeLinecap="round"/>
                    <line x1="28" y1="124" x2="152" y2="124" stroke="#e2e8f0" strokeWidth="1"/>
                    <line x1="28" y1="188" x2="152" y2="188" stroke="#e2e8f0" strokeWidth="1"/>
                  </svg>

                  {/* Tray Labels overlay */}
                  <div className="tray-overlays">
                    {trayEntries.map((t, i) => (
                      <div key={t.key} className={`tray-label-overlay tray-pos-${i + 1}`}>
                        {assignment[t.key] !== null ? (
                          <div className="tray-assigned">
                            <span className="tray-assigned-text">Table{assignment[t.key]}</span>
                            <button className="tray-remove-btn" onClick={() => removeTray(t.key)}>
                              <X className="icon-xxs" />
                            </button>
                          </div>
                        ) : (
                          <div className="tray-empty">
                            <span className="tray-empty-text">— ว่าง —</span>
                          </div>
                        )}
                      </div>
                    ))}
                  </div>
                </div>
              </div>

              {/* CENTER — Table Selector */}
              <div className="table-selector">
                <div className="table-hint">แตะโต๊ะเพื่อกำหนดชั้นวาง (เรียงลำดับการส่ง)</div>
                <div className="table-stagger">
                  {TABLES.map((t, i) => {
                    const isAssigned = assignedTables.includes(t);
                    const orderNum = assignedTables.indexOf(t) + 1;
                    return (
                      <button
                        key={t}
                        onClick={() => assignTable(t)}
                        className={`table-chip ${isAssigned ? 'table-chip--assigned' : ''} table-chip--pos-${i}`}
                      >
                        {isAssigned && <span className="chip-order">{orderNum}</span>}
                        <span className="chip-label">Table{t}</span>
                        {isAssigned && <CheckCircle className="chip-check" />}
                      </button>
                    );
                  })}
                </div>
              </div>

              {/* RIGHT SIDEBAR */}
              <div className="right-sidebar">
                <button className="sidebar-btn"><Settings className="sidebar-icon" /></button>
                <button className="sidebar-btn sidebar-btn--green">
                  <span className="sidebar-dot" />
                </button>
                <button className="sidebar-btn">
                  <Navigation className="sidebar-icon" />
                </button>
              </div>
            </div>
          )}

          {/* ══════ NAVIGATING ══════ */}
          {fsmState === 'NAVIGATING' && (
            <div className="fullview nav-view">
              <RobotEyes mood="navigating" />
              <div className="nav-glow-ring">
                <div className="nav-inner-ring">
                  <Navigation className="nav-icon" />
                </div>
              </div>
              <h2 className="nav-title">กำลังเดินทาง...</h2>
              <p className="nav-dest">
                ปลายทาง: <strong>โต๊ะ {currentTable}</strong>
                {remainingTables.length > 0 && (
                  <span className="nav-next"> → ต่อไปโต๊ะ {remainingTables.join(' → ')}</span>
                )}
              </p>
              <div className="nav-bar-wrap">
                <div className="nav-bar-labels">
                  <span>{queueIndex === 0 ? 'ครัว' : `โต๊ะ ${queue[queueIndex - 1]}`}</span>
                  <span className="nav-pct">{navProgress}%</span>
                  <span>โต๊ะ {currentTable}</span>
                </div>
                <div className="nav-bar-track">
                  <div className="nav-bar-fill" style={{ width: `${navProgress}%` }} />
                  <div className="nav-robot-dot" style={{ left: `calc(${navProgress}% - 10px)` }} />
                </div>
              </div>
            </div>
          )}

          {/* ══════ ARRIVED ══════ */}
          {fsmState === 'ARRIVED' && (
            <div className="fullview arrived-view">
              <RobotEyes mood="arrived" />
              <div className="arrived-top">
                <div className="arrived-badge">
                  <MapPin className="icon-sm" />
                  ถึงแล้ว!
                </div>
                <h2 className="arrived-title">
                  สวัสดีครับ <span>โต๊ะ {currentTable}</span>
                </h2>
                <p className="arrived-sub">
                  โปรดหยิบอาหารจาก{' '}
                  <strong>
                    {currentTrayKey === 'tray1' ? 'ชั้น 1 (TOP)' : currentTrayKey === 'tray2' ? 'ชั้น 2 (MID)' : 'ชั้น 3 (BOT)'}
                  </strong>
                  {remainingTables.length > 0 && (
                    <span className="arrived-next"> · ต่อไปจะส่งโต๊ะ {remainingTables.join(', ')}</span>
                  )}
                </p>
              </div>

              <div className="arrived-trays">
                {trayEntries.map(t => {
                  const isCurrent = assignment[t.key] === currentTable;
                  const isAssigned = assignment[t.key] !== null;
                  return (
                    <div
                      key={t.key}
                      className={`arrived-tray ${
                        isCurrent ? 'arrived-tray--current'
                        : isAssigned ? 'arrived-tray--active'
                        : 'arrived-tray--empty'
                      }`}
                    >
                      <div className="arrived-tray-left">
                        <span>{t.label}</span>
                      </div>
                      {isAssigned
                        ? <span className={`arrived-tray-table ${isCurrent ? 'arrived-tray-table--current' : ''}`}>
                            Table {assignment[t.key]}
                            {isCurrent && <span className="pickup-now"> ← หยิบ</span>}
                          </span>
                        : <span className="arrived-tray-empty-tag">ว่าง</span>
                      }
                    </div>
                  );
                })}
              </div>

              <button onClick={() => setFsmState('THANK_YOU')} className="action-btn action-btn--green">
                <CheckCircle className="icon-md" />
                หยิบอาหารเรียบร้อยแล้ว
                {remainingTables.length > 0
                  ? ` · ต่อไปโต๊ะ ${remainingTables[0]}`
                  : ' · กลับครัว'}
              </button>
            </div>
          )}

          {/* ══════ THANK YOU ══════ */}
          {fsmState === 'THANK_YOU' && (
            <div className="fullview thankyou-view">
              <RobotEyes mood="thankyou" />
              <div className="ty-hearts">
                <span>♥</span><span>♥</span>
              </div>
              <h2 className="ty-heading">ขอบคุณครับ! โต๊ะ {currentTable}</h2>
              <p className="ty-sub">
                {remainingTables.length > 0
                  ? `กำลังไปส่งโต๊ะ ${remainingTables[0]} ต่อไป...`
                  : 'ส่งอาหารครบทุกโต๊ะแล้ว! กำลังกลับครัว...'}
              </p>
              <div className="ty-countdown">
                <div className="ty-bar" style={{ width: `${(thankYouSeconds / 4) * 100}%` }} />
                <span className="ty-label">
                  <Timer className="icon-xs" />
                  {remainingTables.length > 0
                    ? <>ไปโต๊ะ {remainingTables[0]} ใน <strong>{thankYouSeconds}</strong> วิ</>
                    : <>กลับครัวใน <strong>{thankYouSeconds}</strong> วิ</>
                  }
                </span>
              </div>
            </div>
          )}

          {/* ══════ RETURNING ══════ */}
          {fsmState === 'RETURNING' && (
            <div className="fullview returning-view">
              <RobotEyes mood="returning" />
              <div className="ret-icon">
                <RotateCcw className="ret-spin-icon" />
              </div>
              <h2 className="ret-title">ส่งครบทุกโต๊ะแล้ว!</h2>
              <p className="ret-sub">กำลังกลับครัว — Returning to kitchen...</p>
              <div className="ret-summary">
                {queue.map(t => (
                  <span key={t} className="ret-done-tag">✓ โต๊ะ {t}</span>
                ))}
              </div>
            </div>
          )}
        </main>

        {/* ── Bottom Bar ── */}
        <div className="bottom-bar">
          {fsmState === 'KITCHEN_CONFIG' && (
            <button
              onClick={handleStart}
              className={`start-btn ${canStart ? 'start-btn--ready' : 'start-btn--disabled'}`}
              disabled={!canStart}
            >
              Start
              <ChevronRight className="start-chevron" />
              <ChevronRight className="start-chevron start-chevron2" />
            </button>
          )}

          {fsmState !== 'KITCHEN_CONFIG' && (
            <div className="sim-bar">
              <span className="sim-label">⚙ SIMULATOR:</span>
              {fsmState === 'NAVIGATING' && (
                <button onClick={() => setFsmState('ARRIVED')} className="sim-chip sim-chip--amber">
                  <MapPin className="icon-xxs" /> ถึงโต๊ะ {currentTable}
                </button>
              )}
              <button onClick={() => {
                setAssignment({ tray1: null, tray2: null });
                setQueue([]);
                setQueueIndex(0);
                setFsmState('KITCHEN_CONFIG');
              }} className="sim-chip sim-chip--blue">
                <Home className="icon-xxs" /> รีเซ็ต
              </button>
            </div>
          )}
        </div>

      </div>
    </div>
  );
}
