use crate::app::App;
use crate::frame::{self, Frame, Primitive};
use ratatui::{
    Frame as TuiFrame,
    layout::{Alignment, Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::{Block, BorderType, Borders, Cell, Paragraph, Row, Table, Wrap},
};

// ── Palette ──────────────────────────────────────────────────────────────────
const ACCENT: Color    = Color::Rgb(0, 200, 180);
const DIM: Color       = Color::Rgb(80, 90, 100);
const TEXT: Color      = Color::Rgb(210, 215, 220);
const TTL_COL: Color   = Color::Rgb(80, 160, 255);
const EVENT_COL: Color = Color::Rgb(255, 200, 60);
const OK_COL: Color    = Color::Rgb(60, 210, 120);
const ALERT_COL: Color = Color::Rgb(255, 80, 80);

// A beacon rebroadcasts every ~3 s; older than this counts as stale.
const STALE_SECS: u64 = 10;

fn label_style() -> Style { Style::default().fg(DIM) }
fn value_style(color: Color) -> Style {
    Style::default().fg(color).add_modifier(Modifier::BOLD)
}
fn accent_block(title: &str) -> Block<'_> {
    Block::default()
        .title(Span::styled(
            format!(" {} ", title),
            Style::default().fg(ACCENT).add_modifier(Modifier::BOLD),
        ))
        .borders(Borders::ALL)
        .border_type(BorderType::Rounded)
        .border_style(Style::default().fg(DIM))
}

pub fn ui(frame: &mut TuiFrame, app: &mut App) {
    let area = frame.area();
    let root = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),
            Constraint::Min(0),
            Constraint::Length(3),
        ])
        .split(area);
    render_header(frame, root[0], app);
    render_body(frame, root[1], app);
    render_footer(frame, root[2], app);
}

fn render_header(frame: &mut TuiFrame, area: Rect, app: &App) {
    let cols = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Fill(1), Constraint::Length(36)])
        .split(area);

    frame.render_widget(
        Paragraph::new(Line::from(vec![
            Span::styled("LIVING", Style::default().fg(ACCENT).add_modifier(Modifier::BOLD)),
            Span::styled(" MAP", Style::default().fg(TEXT).add_modifier(Modifier::BOLD)),
            Span::styled(format!("  /  ESP-NOW mesh monitor  /  {}", app.port_name), Style::default().fg(DIM)),
        ]))
        .block(Block::default().borders(Borders::ALL).border_type(BorderType::Rounded).border_style(Style::default().fg(DIM)))
        .alignment(Alignment::Left),
        cols[0],
    );

    let status_color = if app.connected { OK_COL } else { ALERT_COL };
    let status_label = if app.connected { "● LIVE" } else { "○ CONNECTING" };
    frame.render_widget(
        Paragraph::new(Line::from(vec![
            Span::styled(status_label, Style::default().fg(status_color).add_modifier(Modifier::BOLD)),
            Span::styled(format!("  rx {}  bad {}", app.message_count, app.bad_count), Style::default().fg(DIM)),
        ]))
        .block(Block::default().borders(Borders::ALL).border_type(BorderType::Rounded).border_style(Style::default().fg(DIM)))
        .alignment(Alignment::Center),
        cols[1],
    );
}

fn render_body(frame: &mut TuiFrame, area: Rect, app: &App) {
    let rows = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Percentage(45), Constraint::Percentage(55)])
        .split(area);
    let cols = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(40), Constraint::Percentage(60)])
        .split(rows[1]);

    render_nodes(frame, rows[0], app);
    render_last_frame(frame, cols[0], app);
    render_log(frame, cols[1], app);
}

fn opt<T: std::fmt::Display>(v: Option<T>) -> String {
    v.map(|v| v.to_string()).unwrap_or_else(|| "—".into())
}

/// TYPE, MSG_ID, SEQ, TTL, EVENT, VALUE cells for a node's most recent frame.
fn frame_cells(last: Option<&Frame>) -> Vec<Cell<'static>> {
    let (kind, seq, ttl, ev, val) = match last {
        Some(Frame::EventBeacon { seq_num, ttl, event_type, event_value, .. }) => (
            "EVENT",
            seq_num.to_string(),
            ttl.to_string(),
            format!("0x{event_type:02x}"),
            event_value.to_string(),
        ),
        Some(f) => (f.kind(), "—".into(), "—".into(), "—".into(), "—".into()),
        None => ("—", "—".into(), "—".into(), "—".into(), "—".into()),
    };
    let msg_id = opt(last.and_then(|f| f.msg_id()).map(|id| format!("0x{id:04x}")));
    vec![
        Cell::from(kind),
        Cell::from(msg_id).style(value_style(TEXT)),
        Cell::from(seq),
        Cell::from(ttl).style(value_style(TTL_COL)),
        Cell::from(ev).style(Style::default().fg(EVENT_COL)),
        Cell::from(val).style(Style::default().fg(EVENT_COL)),
    ]
}

fn render_nodes(frame: &mut TuiFrame, area: Rect, app: &App) {
    let block = accent_block("NODES  (bridge = sent by us · others = heard by the bridge)");
    if app.devices.is_empty() && app.bridge.is_none() {
        let inner = block.inner(area);
        frame.render_widget(block, area);
        frame.render_widget(
            Paragraph::new(Span::styled("no frames yet — waiting for RX lines from the bridge", label_style())),
            inner,
        );
        return;
    }

    let header = Row::new(
        ["", "MAC", "ROLE", "LAST SEEN", "TYPE", "MSG_ID", "SEQ", "TTL", "EVENT", "VALUE", "IDS", "FRAMES", "BAD"]
            .map(|h| Cell::from(h).style(label_style())),
    );

    // Pinned first row: the bridge itself (it never hears its own broadcasts).
    let bridge_row = app.bridge.as_ref().map(|b| {
        let dot_col = if app.connected { ACCENT } else { ALERT_COL };
        let seen = match b.last_tx {
            Some(t) => format!("tx {}s ago", t.elapsed().as_secs()),
            None => format!("listening (up {}s)", b.ready_at.elapsed().as_secs()),
        };
        let mut cells = vec![
            Cell::from("◆").style(Style::default().fg(dot_col)),
            Cell::from(b.mac.clone()).style(value_style(ACCENT)),
            Cell::from("bridge").style(value_style(ACCENT)),
            Cell::from(seen),
        ];
        cells.extend(frame_cells(b.last.as_ref()));
        cells.extend([
            Cell::from(b.distinct_ids.len().to_string()).style(value_style(ACCENT)),
            Cell::from(b.tx_count.to_string()),
            Cell::from(b.errors.to_string()).style(Style::default().fg(if b.errors > 0 { ALERT_COL } else { DIM })),
        ]);
        Row::new(cells)
    });

    let node_rows = app.devices.iter().map(|(mac, d)| {
        let age = d.last_seen.elapsed().as_secs();
        let (dot, dot_col) = if age < STALE_SECS { ("●", OK_COL) } else { ("○", ALERT_COL) };
        let mut cells = vec![
            Cell::from(dot).style(Style::default().fg(dot_col)),
            Cell::from(mac.clone()).style(Style::default().fg(TEXT)),
            Cell::from("node").style(label_style()),
            Cell::from(format!("{:.1}s  ({}s ago)", d.bridge_millis as f64 / 1000.0, age)),
        ];
        cells.extend(frame_cells(d.last.as_ref()));
        cells.extend([
            Cell::from(d.distinct_ids.len().to_string()).style(value_style(ACCENT)),
            Cell::from(d.frames.to_string()),
            Cell::from(d.bad.to_string()).style(Style::default().fg(if d.bad > 0 { ALERT_COL } else { DIM })),
        ]);
        Row::new(cells)
    });

    let widths = [
        Constraint::Length(1),
        Constraint::Length(17),
        Constraint::Length(6),
        Constraint::Length(22),
        Constraint::Length(13),
        Constraint::Length(7),
        Constraint::Length(4),
        Constraint::Length(4),
        Constraint::Length(6),
        Constraint::Length(6),
        Constraint::Length(4),
        Constraint::Length(7),
        Constraint::Length(4),
    ];
    frame.render_widget(
        Table::new(bridge_row.into_iter().chain(node_rows), widths).header(header).block(block).column_spacing(1),
        area,
    );
}

fn describe(f: &Frame) -> Vec<Line<'static>> {
    let kv = |k: &str, v: String, c: Color| {
        Line::from(vec![
            Span::styled(format!("{k:<14}"), label_style()),
            Span::styled(v, value_style(c)),
        ])
    };
    let mut lines = vec![kv("frame_type", f.kind().to_string(), ACCENT)];
    match f {
        Frame::EventBeacon { msg_id, seq_num, ttl, timestamp, event_type, event_value, primitives } => {
            lines.push(kv("msg_id", format!("0x{msg_id:04x}"), TEXT));
            lines.push(kv("seq_num", seq_num.to_string(), TEXT));
            lines.push(kv("ttl", ttl.to_string(), TTL_COL));
            lines.push(kv("timestamp", format!("{timestamp} s"), TEXT));
            lines.push(kv("event_type", format!("0x{event_type:02x} {}", frame::event_name(*event_type)), EVENT_COL));
            lines.push(kv("event_value", event_value.to_string(), EVENT_COL));
            let prims: Vec<String> = primitives
                .iter()
                .map(|p| match p {
                    Primitive::Forward(cm) => format!("F{cm}cm"),
                    Primitive::Turn(deg) => format!("T{deg:+}°"),
                    Primitive::Unknown(t, m) => format!("?{t}:{m}"),
                })
                .collect();
            lines.push(kv("primitives", format!("[{}] {}", primitives.len(), prims.join(" ")), TEXT));
        }
        Frame::GoTrigger { msg_id, mission_code } => {
            lines.push(kv("msg_id", format!("0x{msg_id:04x}"), TEXT));
            lines.push(kv("mission_code", mission_code.to_string(), TEXT));
        }
        Frame::StatusUpdate { msg_id, payload_len } => {
            lines.push(kv("msg_id", format!("0x{msg_id:04x}"), TEXT));
            lines.push(kv("payload", format!("{payload_len} B (reserved, not decoded)"), TEXT));
        }
        Frame::Ack => {}
        Frame::Unknown { frame_type } => lines.push(kv("raw type", format!("0x{frame_type:02x}"), ALERT_COL)),
    }
    lines
}

fn render_last_frame(frame: &mut TuiFrame, area: Rect, app: &App) {
    let block = accent_block("LAST FRAME");
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let Some(entry) = app.log.front() else {
        frame.render_widget(Paragraph::new(Span::styled("waiting for frames...", label_style())), inner);
        return;
    };

    let mut lines = vec![
        Line::from(vec![
            Span::styled(format!("{:<14}", if entry.tx { "sent by" } else { "from" }), label_style()),
            Span::styled(
                if entry.tx { format!("{} (bridge)", entry.mac) } else { entry.mac.clone() },
                value_style(if entry.tx { ACCENT } else { TEXT }),
            ),
        ]),
        Line::from(vec![
            Span::styled(format!("{:<14}", "bridge time"), label_style()),
            Span::styled(format!("{} ms", entry.bridge_millis), Style::default().fg(TEXT)),
        ]),
    ];
    match &entry.decoded {
        Ok(f) => lines.extend(describe(f)),
        Err(e) => lines.push(Line::from(Span::styled(format!("DECODE ERROR: {e}"), value_style(ALERT_COL)))),
    }
    lines.push(Line::from(""));
    lines.push(Line::from(Span::styled(entry.hex.clone(), label_style())));

    frame.render_widget(Paragraph::new(lines).wrap(Wrap { trim: false }), inner);
}

fn render_log(frame: &mut TuiFrame, area: Rect, app: &App) {
    let block = accent_block("FRAME LOG");
    let inner = block.inner(area);
    frame.render_widget(block, area);

    let lines: Vec<Line> = app
        .log
        .iter()
        .take(inner.height as usize)
        .map(|e| {
            let (summary, color) = match &e.decoded {
                Ok(Frame::EventBeacon { msg_id, seq_num, ttl, event_type, event_value, .. }) => (
                    format!(
                        "EVENT id=0x{msg_id:04x} seq={seq_num} ttl={ttl} ev=0x{event_type:02x} {} val={event_value}",
                        frame::event_name(*event_type)
                    ),
                    TEXT,
                ),
                Ok(f) => (format!("{} id={}", f.kind(), opt(f.msg_id().map(|i| format!("0x{i:04x}")))), TEXT),
                Err(err) => (format!("BAD {err}  {}", e.hex), ALERT_COL),
            };
            Line::from(vec![
                Span::styled(format!("{:>9.3} ", e.bridge_millis as f64 / 1000.0), label_style()),
                Span::styled(if e.tx { "TX " } else { "RX " }, Style::default().fg(if e.tx { ACCENT } else { DIM })),
                Span::styled(format!("{} ", &e.mac[e.mac.len().saturating_sub(8)..]), Style::default().fg(ACCENT)),
                Span::styled(summary, Style::default().fg(color)),
            ])
        })
        .collect();
    frame.render_widget(Paragraph::new(lines), inner);
}

fn render_footer(frame: &mut TuiFrame, area: Rect, app: &App) {
    frame.render_widget(
        Paragraph::new(Line::from(vec![
            Span::styled(" q ", Style::default().fg(Color::Black).bg(DIM)),
            Span::styled("  quit  ", label_style()),
            Span::styled(" c ", Style::default().fg(Color::Black).bg(DIM)),
            Span::styled("  clear", label_style()),
            Span::styled("     bridge: ", label_style()),
            Span::styled(app.bridge_status.clone(), Style::default().fg(TEXT)),
        ]))
        .block(Block::default()
            .borders(Borders::ALL)
            .border_type(BorderType::Rounded)
            .border_style(Style::default().fg(DIM))),
        area,
    );
}
