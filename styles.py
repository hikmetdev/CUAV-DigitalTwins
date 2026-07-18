# ----------------------------------------------------------------------
# QSS Stylesheet - Palantir/Anduril Defense UI Theme (Dark Navy/Green)
# ----------------------------------------------------------------------
DARK_THEME_QSS = """
QWidget {
    background-color: #030712;
    color: #e2e8f0;
    font-family: "Inter", "Segoe UI", sans-serif;
}

QFrame.panel {
    background-color: #090d16;
    border: 1px solid #1e293b;
    border-radius: 6px;
}

QFrame.panel-header {
    background-color: #0d1522;
    border-bottom: 1px solid #1e293b;
    border-top-left-radius: 6px;
    border-top-right-radius: 6px;
}

QLabel.title-label {
    color: #e2e8f0;
    font-weight: bold;
    font-size: 11px;
    font-family: "Roboto Mono", "Courier New";
}

QLabel.badge-green {
    background-color: rgba(16, 185, 129, 0.1);
    color: #10b981;
    border: 1px solid rgba(16, 185, 129, 0.3);
    border-radius: 3px;
    padding: 2px 6px;
    font-size: 9px;
    font-weight: bold;
    font-family: "Roboto Mono";
}

QLabel.badge-cyan {
    background-color: rgba(6, 182, 212, 0.1);
    color: #06b6d4;
    border: 1px solid rgba(6, 182, 212, 0.3);
    border-radius: 3px;
    padding: 2px 6px;
    font-size: 9px;
    font-weight: bold;
    font-family: "Roboto Mono";
}

QLabel.badge-warn {
    background-color: rgba(245, 158, 11, 0.1);
    color: #f59e0b;
    border: 1px solid rgba(245, 158, 11, 0.3);
    border-radius: 3px;
    padding: 2px 6px;
    font-size: 9px;
    font-weight: bold;
    font-family: "Roboto Mono";
}

/* Table Widget styling */
QTableWidget {
    background-color: #090d16;
    gridline-color: rgba(30, 41, 59, 0.3);
    border: none;
    font-family: "Roboto Mono", monospace;
    font-size: 10px;
}

QTableWidget::item {
    padding: 6px;
    border-bottom: 1px solid rgba(30, 41, 59, 0.2);
}

QTableWidget::item:selected {
    background-color: rgba(6, 182, 212, 0.15);
    color: #06b6d4;
}

QHeaderView::section {
    background-color: #0b111c;
    color: #94a3b8;
    border: none;
    border-bottom: 1px solid #1e293b;
    font-weight: bold;
    font-size: 10px;
    padding: 4px;
}

/* Collapsible Console Terminal */
QPlainTextEdit.console {
    background-color: #010409;
    border: 1px solid #1e293b;
    color: #10b981;
    font-family: "Roboto Mono", monospace;
    font-size: 10px;
}

/* Chat text browser */
QTextBrowser.chat-history {
    background-color: #060910;
    border: 1px solid #1e293b;
    border-radius: 4px;
    color: #e2e8f0;
    font-size: 11px;
}

/* Inputs and Forms */
QLineEdit.chat-input {
    background-color: #010409;
    border: 1px solid #1e293b;
    border-radius: 4px;
    padding: 6px 10px;
    color: #f1f5f9;
    font-size: 11px;
    font-family: "Roboto Mono";
}

QLineEdit.chat-input:focus {
    border: 1px solid #10b981;
}

/* Buttons */
QPushButton.btn-primary {
    background-color: #10b981;
    color: #030712;
    border: none;
    border-radius: 4px;
    font-weight: bold;
    padding: 6px 12px;
    font-size: 11px;
}

QPushButton.btn-primary:hover {
    background-color: #059669;
}

QPushButton.btn-secondary {
    background-color: rgba(30, 41, 59, 0.4);
    color: #e2e8f0;
    border: 1px solid #1e293b;
    border-radius: 4px;
    font-size: 11px;
}

QPushButton.btn-secondary:hover {
    background-color: rgba(30, 41, 59, 0.8);
    border: 1px solid #10b981;
}

QPushButton.suggestion-pill {
    background-color: rgba(30, 41, 59, 0.3);
    color: #94a3b8;
    border: 1px solid rgba(30, 41, 59, 0.6);
    border-radius: 10px;
    padding: 3px 8px;
    font-size: 9px;
    font-family: "Roboto Mono";
}

QPushButton.suggestion-pill:hover {
    background-color: rgba(6, 182, 212, 0.1);
    color: #06b6d4;
    border: 1px solid rgba(6, 182, 212, 0.4);
}

/* Scrollbars */
QScrollBar:vertical {
    border: none;
    background: #030712;
    width: 6px;
    margin: 0px;
}

QScrollBar::handle:vertical {
    background: #1e293b;
    min-height: 20px;
    border-radius: 3px;
}

QScrollBar::handle:vertical:hover {
    background: #10b981;
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    border: none;
    background: none;
}
"""

