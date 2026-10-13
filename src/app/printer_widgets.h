#pragma once
#include <QCheckBox>
#include <QPainter>
#include <QPainterPath>
// Keep checkbox states legible when a dark system theme is used with a light printer editor.
class PrinterCheckBox : public QCheckBox {
  public:
    explicit PrinterCheckBox(const QString &text = {}) : QCheckBox(text) {
        setMinimumHeight(22);
        setMinimumWidth(text.isEmpty() ? 22 : 48);
    }

  protected:
    void paintEvent(QPaintEvent *) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing);
        QRectF box(1, (height() - 16) / 2.0, 16, 16);
        p.setPen(QColor("#9ca8ae"));
        p.setBrush(isChecked() ? (isEnabled() ? QColor("#00b352") : QColor("#9ca8ae")) : QColor("#ffffff"));
        p.drawRoundedRect(box, 2, 2);
        if (isChecked()) {
            QPainterPath check;
            check.moveTo(box.left() + 3, box.top() + 8);
            check.lineTo(box.left() + 7, box.top() + 12);
            check.lineTo(box.left() + 13, box.top() + 4);
            p.setPen(QPen(Qt::white, 1.8));
            p.drawPath(check);
        }
        p.setPen(isEnabled() ? QColor("#30383c") : QColor("#879196"));
        p.drawText(rect().adjusted(24, 0, 0, 0), Qt::AlignVCenter | Qt::AlignLeft, text());
        if (hasFocus()) {
            p.setBrush(Qt::NoBrush);
            p.setPen(QPen(QColor("#007c39"), 1, Qt::DotLine));
            p.drawRect(rect().adjusted(0, 0, -1, -1));
        }
    }
};
