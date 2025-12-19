#include <QMainWindow>
#include <QApplication>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QSlider>
#include <QGroupBox>
#include <QGridLayout>
#include <QThread>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QLineEdit>
#include <QDialog>
#include <QScrollArea>
#include <QPainter>
#include <QWheelEvent>
#include <opencv2/opencv.hpp>
#include <cmath>

const double SCALE = 0.5;

// Hilfsfunktionen
bool is_image_file(const QString &path) {
    QStringList ext = {"jpg", "jpeg", "png", "bmp", "tif", "tiff", "webp"};
    return ext.contains(QFileInfo(path).suffix().toLower());
}

QImage mat_to_qimage(const cv::Mat &mat) {
    if(mat.type() == CV_8UC1) {
        return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8).copy();
    } else if(mat.type() == CV_8UC3) {
        cv::Mat rgb;
        cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
        return QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888).copy();
    }
    return QImage();
}

// Drag & Drop Label
class DropImageLabel : public QLabel {
    Q_OBJECT
public:
    explicit DropImageLabel(QWidget *parent = nullptr) : QLabel(parent) { 
        setAcceptDrops(true); 
    }
signals:
    void fileDropped(const QString &path);
protected:
    void dragEnterEvent(QDragEnterEvent *event) override {
        if (event->mimeData()->hasUrls()) {
            for(const auto &url : event->mimeData()->urls()) {
                if(is_image_file(url.toLocalFile())) {
                    event->acceptProposedAction();
                    return;
                }
            }
        }
    }
    void dropEvent(QDropEvent *event) override {
        for(const auto &url : event->mimeData()->urls()) {
            QString path = url.toLocalFile();
            if(is_image_file(path)) {
                emit fileDropped(path);
                event->acceptProposedAction();
                return;
            }
        }
    }
};

// Kalibrierungs-Dialog
class CalibrationDialog : public QDialog {
    Q_OBJECT
public:
    std::vector<cv::Point> calibration_points;
    
    CalibrationDialog(const cv::Mat &image, QWidget *parent = nullptr) 
        : QDialog(parent), original_image(image), zoom_factor(1.0) {
        setWindowTitle("Calibration - Select 2 points");
        resize(1200, 800);
        
        auto *layout = new QVBoxLayout(this);
        
        info_label = new QLabel("Click 2 points with known distance");
        info_label->setStyleSheet("background-color: #FF9800; color: white; font-size: 16px; padding: 15px;");
        info_label->setAlignment(Qt::AlignCenter);
        layout->addWidget(info_label);
        
        auto *zoom_layout = new QHBoxLayout();
        auto *btn_zoom_in = new QPushButton("Zoom +");
        auto *btn_zoom_out = new QPushButton("Zoom -");
        auto *btn_zoom_fit = new QPushButton("Fit");
        auto *btn_reset = new QPushButton("Reset");
        auto *btn_cancel = new QPushButton("Cancel");
        
        zoom_label = new QLabel("100%");
        
        connect(btn_zoom_in, &QPushButton::clicked, this, &CalibrationDialog::zoom_in);
        connect(btn_zoom_out, &QPushButton::clicked, this, &CalibrationDialog::zoom_out);
        connect(btn_zoom_fit, &QPushButton::clicked, this, &CalibrationDialog::zoom_fit);
        connect(btn_reset, &QPushButton::clicked, this, &CalibrationDialog::reset_points);
        connect(btn_cancel, &QPushButton::clicked, this, &QDialog::reject);
        
        zoom_layout->addWidget(btn_zoom_in);
        zoom_layout->addWidget(btn_zoom_out);
        zoom_layout->addWidget(btn_zoom_fit);
        zoom_layout->addWidget(zoom_label);
        zoom_layout->addStretch();
        zoom_layout->addWidget(btn_reset);
        zoom_layout->addWidget(btn_cancel);
        layout->addLayout(zoom_layout);
        
        auto *scroll = new QScrollArea();
        scroll->setWidgetResizable(true);
        image_label = new QLabel();
        image_label->setAlignment(Qt::AlignCenter);
        scroll->setWidget(image_label);
        layout->addWidget(scroll);
        
        finish_container = new QWidget();
        finish_layout = new QVBoxLayout(finish_container);
        layout->addWidget(finish_container);
        finish_container->hide();
        
        display_image();
    }
    
protected:
    void mousePressEvent(QMouseEvent *event) override {
        if(event->button() == Qt::LeftButton && calibration_points.size() < 2) {
            QPoint pos = image_label->mapFrom(this, event->pos());
            int x = pos.x() / zoom_factor;
            int y = pos.y() / zoom_factor;
            
            if(x >= 0 && x < original_image.cols && y >= 0 && y < original_image.rows) {
                calibration_points.push_back(cv::Point(x, y));
                display_image();
                
                if(calibration_points.size() == 1) {
                    info_label->setText("Point 1 set - Click point 2");
                } else if(calibration_points.size() == 2) {
                    info_label->setText("Both points set - Click Done");
                    show_finish_button();
                }
            }
        }
    }
    
    void wheelEvent(QWheelEvent *event) override {
        if(event->angleDelta().y() > 0) zoom_in();
        else zoom_out();
    }
    
private slots:
    void zoom_in() {
        zoom_factor = std::min(zoom_factor * 1.5, 8.0);
        display_image();
    }
    
    void zoom_out() {
        zoom_factor = std::max(zoom_factor / 1.5, 0.2);
        display_image();
    }
    
    void zoom_fit() {
        zoom_factor = 1.0;
        display_image();
    }
    
    void reset_points() {
        calibration_points.clear();
        finish_container->hide();
        QLayoutItem *item;
        while((item = finish_layout->takeAt(0)) != nullptr) {
            delete item->widget();
            delete item;
        }
        display_image();
        info_label->setText("Click 2 points with known distance");
    }
    
    void finish_calibration() {
        if(calibration_points.size() == 2) accept();
    }
    
private:
    cv::Mat original_image;
    double zoom_factor;
    QLabel *image_label, *info_label, *zoom_label;
    QWidget *finish_container;
    QVBoxLayout *finish_layout;
    
    void display_image() {
        int new_w = original_image.cols * zoom_factor;
        int new_h = original_image.rows * zoom_factor;
        
        cv::Mat display_img;
        cv::resize(original_image, display_img, cv::Size(new_w, new_h));
        
        if(display_img.channels() == 1) {
            cv::cvtColor(display_img, display_img, cv::COLOR_GRAY2RGB);
        }
        
        QImage qimg = mat_to_qimage(display_img);
        QPixmap pixmap = QPixmap::fromImage(qimg);
        
        if(!calibration_points.empty()) {
            QPainter painter(&pixmap);
            painter.setRenderHint(QPainter::Antialiasing);
            
            QPen pen(QColor(255, 0, 0), std::max(4, (int)(6 * zoom_factor)));
            painter.setPen(pen);
            
            for(const auto &pt : calibration_points) {
                int x = pt.x * zoom_factor;
                int y = pt.y * zoom_factor;
                int radius = 12 * zoom_factor;
                painter.drawEllipse(x - radius, y - radius, radius * 2, radius * 2);
                
                int size = 20 * zoom_factor;
                painter.drawLine(x - size, y, x + size, y);
                painter.drawLine(x, y - size, x, y + size);
            }
            
            if(calibration_points.size() == 2) {
                pen.setColor(QColor(0, 255, 0));
                pen.setWidth(std::max(3, (int)(4 * zoom_factor)));
                painter.setPen(pen);
                
                int x1 = calibration_points[0].x * zoom_factor;
                int y1 = calibration_points[0].y * zoom_factor;
                int x2 = calibration_points[1].x * zoom_factor;
                int y2 = calibration_points[1].y * zoom_factor;
                painter.drawLine(x1, y1, x2, y2);
                
                double dist = cv::norm(calibration_points[1] - calibration_points[0]);
                painter.setPen(QColor(255, 255, 0));
                QFont font = painter.font();
                font.setPointSize(std::max(14, (int)(18 * zoom_factor)));
                font.setBold(true);
                painter.setFont(font);
                painter.drawText((x1 + x2)/2 + 15, (y1 + y2)/2 - 15, 
                    QString("%1 px").arg(dist, 0, 'f', 1));
            }
        }
        
        image_label->setPixmap(pixmap);
        zoom_label->setText(QString("%1%").arg((int)(zoom_factor * 100)));
    }
    
    void show_finish_button() {
        auto *btn = new QPushButton("Finish calibration");
        btn->setStyleSheet("background-color: #4CAF50; color: white; font-size: 18px; padding: 15px; font-weight: bold;");
        connect(btn, &QPushButton::clicked, this, &CalibrationDialog::finish_calibration);
        finish_layout->addWidget(btn);
        finish_container->show();
    }
};

// Worker-Thread für Bildverarbeitung
class ImageProcessingWorker : public QThread {
    Q_OBJECT
public:
    ImageProcessingWorker(const cv::Mat &proc, const cv::Mat &full, 
                          int cLow, int cHigh, double p2mm)
        : proc_color(proc.clone()), full_color(full.clone()),
          canny_low(cLow), canny_high(cHigh), pixel_to_mm(p2mm) {}
    
    void run() override {
        try {
            cv::Mat gray, blurred, bw;
            cv::cvtColor(proc_color, gray, cv::COLOR_BGR2GRAY);
            
            double s_gauss = std::max(0.5, 3.0 * SCALE);
            cv::GaussianBlur(gray, blurred, cv::Size(0,0), s_gauss);
            cv::Canny(blurred, bw, canny_low, canny_high, 3, true);
            
            auto disk = [](int r) { 
                return cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2*r+1, 2*r+1)); 
            };
            
            // Morphologie
            cv::morphologyEx(bw, bw, cv::MORPH_CLOSE, disk(std::max(1, (int)(15 * SCALE))));
            
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(bw, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            bw = cv::Mat::zeros(bw.size(), CV_8UC1);
            cv::drawContours(bw, contours, -1, 255, -1);
            
            cv::morphologyEx(bw, bw, cv::MORPH_OPEN, disk(std::max(1, (int)(8 * SCALE))));
            
            // Größte Komponente
            cv::Mat labels, stats, centroids;
            int n = cv::connectedComponentsWithStats(bw, labels, stats, centroids);
            if(n > 1) {
                int max_idx = 1;
                int max_area = 0;
                for(int i=1; i<n; ++i) {
                    int area = stats.at<int>(i, cv::CC_STAT_AREA);
                    if(area > max_area) { max_area = area; max_idx = i; }
                }
                bw = (labels == max_idx);
            }
            
            cv::morphologyEx(bw, bw, cv::MORPH_CLOSE, disk(std::max(1, (int)(5 * SCALE))));
            
            // Finale Kontur
            cv::findContours(bw, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            if(contours.empty()) {
                emit finished("No objects found", mat_to_qimage(full_color), 
                             mat_to_qimage(bw), mat_to_qimage(full_color));
                return;
            }
            
            auto it = std::max_element(contours.begin(), contours.end(), 
                [](const auto& a, const auto& b){ return cv::contourArea(a) < cv::contourArea(b); });
            
            std::vector<cv::Point> best_cnt = *it;
            cv::approxPolyDP(best_cnt, best_cnt, 0.005 * cv::arcLength(best_cnt, true), true);
            
            double area_px = cv::contourArea(best_cnt);
            double perim_px = cv::arcLength(best_cnt, true);
            double area_mm2 = area_px * std::pow(pixel_to_mm, 2);
            double perim_mm = perim_px * pixel_to_mm;
            
            // Auf Full-Res skalieren
            double scale_x = (double)full_color.cols / proc_color.cols;
            double scale_y = (double)full_color.rows / proc_color.rows;
            
            std::vector<cv::Point> full_cnt;
            for(const auto &p : best_cnt) {
                full_cnt.push_back(cv::Point(p.x * scale_x, p.y * scale_y));
            }
            
            cv::Mat result = full_color.clone();
            cv::drawContours(result, std::vector<std::vector<cv::Point>>{full_cnt}, -1, 
                           cv::Scalar(0, 0, 255), 3);
            
            QString info = QString("Area: %1 mm² | Perimeter: %2 mm")
                .arg(area_mm2, 0, 'f', 2).arg(perim_mm, 0, 'f', 2);
            
            int text_h = result.rows * 0.03;
            cv::putText(result, info.toStdString(), cv::Point(20, result.rows - 50),
                       cv::FONT_HERSHEY_SIMPLEX, text_h / 30.0, cv::Scalar(255, 255, 0), 2);
            
            QString result_text = QString("1 Object found\nCalibration: 1 Pixel = %1 mm\n\nArea: %2 mm²\nPerimeter: %3 mm")
                .arg(pixel_to_mm, 0, 'f', 4).arg(area_mm2, 0, 'f', 2).arg(perim_mm, 0, 'f', 2);
            
            emit finished(result_text, mat_to_qimage(full_color), 
                         mat_to_qimage(bw), mat_to_qimage(result));
        } catch(...) {
            emit error("Processing failed");
        }
    }
    
signals:
    void finished(QString text, QImage orig, QImage edge, QImage result);
    void error(QString msg);
    
private:
    cv::Mat proc_color, full_color;
    int canny_low, canny_high;
    double pixel_to_mm;
};

// Hauptfenster
class SpiegelMessApp : public QMainWindow {
    Q_OBJECT
public:
    SpiegelMessApp() {
        setWindowTitle("Mirror Measurement");
        setGeometry(100, 100, 1400, 900);
        setAcceptDrops(true);
        
        init_ui();
    }
    
protected:
    void dragEnterEvent(QDragEnterEvent *event) override {
        if(event->mimeData()->hasUrls()) {
            for(const auto &url : event->mimeData()->urls()) {
                if(is_image_file(url.toLocalFile())) {
                    event->acceptProposedAction();
                    return;
                }
            }
        }
    }
    
    void dropEvent(QDropEvent *event) override {
        for(const auto &url : event->mimeData()->urls()) {
            QString path = url.toLocalFile();
            if(is_image_file(path)) {
                load_image_from_path(path);
                event->acceptProposedAction();
                return;
            }
        }
    }
    
private slots:
    void upload_image() {
        if(processing) return;
        QString path = QFileDialog::getOpenFileName(this, "Select image", "", 
            "Images (*.jpg *.jpeg *.png *.bmp)");
        if(!path.isEmpty()) load_image_from_path(path);
    }
    
    void load_image_from_path(const QString &path) {
        if(processing || !QFile::exists(path) || !is_image_file(path)) return;
        
        full_res_color = cv::imread(path.toStdString());
        if(full_res_color.empty()) {
            QMessageBox::critical(this, "Error", "Could not load image");
            return;
        }
        
        cv::resize(full_res_color, current_color, cv::Size(), SCALE, SCALE, cv::INTER_AREA);
        cv::cvtColor(current_color, current_gray, cv::COLOR_BGR2GRAY);
        
        QPixmap pix = QPixmap::fromImage(mat_to_qimage(full_res_color));
        orig_display->setPixmap(pix.scaled(orig_display->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
        
        btn_calibrate->setEnabled(true);
        btn_process->setEnabled(true);
        btn_save->setEnabled(false);
        
        status_label->setText(QString("Image loaded: %1 (SCALE=%2) - Calibration recommended")
            .arg(QFileInfo(path).fileName()).arg(SCALE));
    }
    
    void start_calibration() {
        if(current_gray.empty()) return;
        
        CalibrationDialog dialog(current_gray, this);
        if(dialog.exec() == QDialog::Accepted && dialog.calibration_points.size() == 2) {
            auto p1 = dialog.calibration_points[0];
            auto p2 = dialog.calibration_points[1];
            double dist_px = cv::norm(p2 - p1);
            
            if(dist_px < 1e-5) {
                QMessageBox::critical(this, "Error", "Points too close");
                return;
            }
            
            double ref_cm = ref_length_input->text().toDouble();
            if(ref_cm <= 0) ref_cm = 1.0;
            
            pixel_to_mm = (ref_cm / dist_px) * 10.0;
            calib_label->setText(QString("%1 mm").arg(pixel_to_mm, 0, 'f', 4));
            
            QString msg = QString("Calibration successful\n%1 Pixel = %2 cm")
                .arg(dist_px, 0, 'f', 1).arg(ref_cm);
            QMessageBox::information(this, "Calibration", msg);
            status_label->setText(msg);
            
            process_image();
        }
    }
    
    void process_image() {
        if(current_color.empty() || full_res_color.empty() || processing) return;
        
        processing = true;
        status_label->setText(QString("Processing image (SCALE=%1)...").arg(SCALE));
        btn_upload->setEnabled(false);
        btn_calibrate->setEnabled(false);
        btn_process->setEnabled(false);
        btn_save->setEnabled(false);
        
        auto *worker = new ImageProcessingWorker(current_color, full_res_color,
            canny_low_slider->value(), canny_high_slider->value(), pixel_to_mm);
        
        connect(worker, &ImageProcessingWorker::finished, this, &SpiegelMessApp::on_finished);
        connect(worker, &ImageProcessingWorker::error, this, &SpiegelMessApp::on_error);
        connect(worker, &QThread::finished, worker, &QObject::deleteLater);
        worker->start();
    }
    
    void on_finished(QString text, QImage orig, QImage edge, QImage result) {
        status_label->setText(text);
        edge_display->setPixmap(QPixmap::fromImage(edge).scaled(edge_display->size(), 
            Qt::KeepAspectRatio, Qt::SmoothTransformation));
        res_display->setPixmap(QPixmap::fromImage(result).scaled(res_display->size(), 
            Qt::KeepAspectRatio, Qt::SmoothTransformation));
        
        last_result_image = result;
        btn_save->setEnabled(true);
        btn_upload->setEnabled(true);
        btn_calibrate->setEnabled(true);
        btn_process->setEnabled(true);
        processing = false;
    }
    
    void on_error(QString msg) {
        status_label->setText(msg);
        btn_upload->setEnabled(true);
        btn_calibrate->setEnabled(true);
        btn_process->setEnabled(true);
        processing = false;
    }
    
    void save_result() {
        if(last_result_image.isNull()) {
            QMessageBox::warning(this, "Save", "No result image yet");
            return;
        }
        
        QString filename = QFileDialog::getSaveFileName(this, "Save result", "",
            "PNG Image (*.png);;JPEG Image (*.jpg *.jpeg)");
        if(!filename.isEmpty()) {
            if(!last_result_image.save(filename)) {
                QMessageBox::critical(this, "Error", "Could not save image");
            }
        }
    }
    
private:
    cv::Mat full_res_color, current_color, current_gray;
    double pixel_to_mm = 0.5;
    bool processing = false;
    QImage last_result_image;
    
    QPushButton *btn_upload, *btn_calibrate, *btn_process, *btn_save;
    QLabel *calib_label, *status_label;
    QLineEdit *ref_length_input;
    QSlider *canny_low_slider, *canny_high_slider;
    QLabel *canny_low_label, *canny_high_label;
    QLabel *orig_display, *edge_display, *res_display;
    
    void init_ui() {
        auto *central = new QWidget();
        setCentralWidget(central);
        auto *layout = new QVBoxLayout(central);
        
        // Buttons
        auto *btn_layout = new QHBoxLayout();
        
        btn_upload = new QPushButton("Picture Upload");
        btn_upload->setStyleSheet("background-color: #4CAF50; color: white; font-size: 14px; padding: 12px;");
        connect(btn_upload, &QPushButton::clicked, this, &SpiegelMessApp::upload_image);
        btn_layout->addWidget(btn_upload);
        
        btn_calibrate = new QPushButton("Calibration");
        btn_calibrate->setStyleSheet("background-color: #FF9800; color: white; font-size: 14px; padding: 12px;");
        btn_calibrate->setEnabled(false);
        connect(btn_calibrate, &QPushButton::clicked, this, &SpiegelMessApp::start_calibration);
        btn_layout->addWidget(btn_calibrate);
        
        btn_process = new QPushButton("Process");
        btn_process->setStyleSheet("background-color: #2196F3; color: white; font-size: 14px; padding: 12px;");
        btn_process->setEnabled(false);
        connect(btn_process, &QPushButton::clicked, this, &SpiegelMessApp::process_image);
        btn_layout->addWidget(btn_process);
        
        btn_save = new QPushButton("Save result");
        btn_save->setStyleSheet("background-color: #9C27B0; color: white; font-size: 14px; padding: 12px;");
        btn_save->setEnabled(false);
        connect(btn_save, &QPushButton::clicked, this, &SpiegelMessApp::save_result);
        btn_layout->addWidget(btn_save);
        
        layout->addLayout(btn_layout);
        
        // Kalibrierung
        auto *calib_layout = new QHBoxLayout();
        calib_layout->addWidget(new QLabel("Reference length (cm):"));
        ref_length_input = new QLineEdit("1.0");
        ref_length_input->setMaximumWidth(80);
        calib_layout->addWidget(ref_length_input);
        calib_layout->addWidget(new QLabel("  |  Calibration: 1 Pixel ="));
        calib_label = new QLabel(QString("%1 mm").arg(pixel_to_mm, 0, 'f', 4));
        calib_label->setStyleSheet("font-weight: bold; color: #2196F3;");
        calib_layout->addWidget(calib_label);
        calib_layout->addStretch();
        layout->addLayout(calib_layout);
        
        // Parameter
        auto *param_group = new QGroupBox("Parameters");
        auto *param_layout = new QGridLayout(param_group);
        
        param_layout->addWidget(new QLabel("Canny low (0-255):"), 0, 0);
        canny_low_slider = new QSlider(Qt::Horizontal);
        canny_low_slider->setRange(0, 255);
        canny_low_slider->setValue(26);
        param_layout->addWidget(canny_low_slider, 0, 1);
        canny_low_label = new QLabel("26");
        param_layout->addWidget(canny_low_label, 0, 2);
        connect(canny_low_slider, &QSlider::valueChanged, [this](int v){ canny_low_label->setText(QString::number(v)); });
        
        param_layout->addWidget(new QLabel("Canny high (0-255):"), 1, 0);
        canny_high_slider = new QSlider(Qt::Horizontal);
        canny_high_slider->setRange(0, 255);
        canny_high_slider->setValue(77);
        param_layout->addWidget(canny_high_slider, 1, 1);
        canny_high_label = new QLabel("77");
        param_layout->addWidget(canny_high_label, 1, 2);
        connect(canny_high_slider, &QSlider::valueChanged, [this](int v){ canny_high_label->setText(QString::number(v)); });
        
        layout->addWidget(param_group);
        
        // Status
        status_label = new QLabel(QString("Ready (SCALE=%1) - Drag & drop images here").arg(SCALE));
        status_label->setStyleSheet("font-size: 13px; padding: 10px; background-color: #f0f0f0;");
        status_label->setWordWrap(true);
        layout->addWidget(status_label);
        
        // Bilder
        auto *img_layout = new QHBoxLayout();
        img_layout->addWidget(create_image_section("Original", true, orig_display));
        img_layout->addWidget(create_image_section("Edges", false, edge_display));
        img_layout->addWidget(create_image_section("Result", false, res_display));
        layout->addLayout(img_layout);
    }
    
    QWidget* create_image_section(const QString &title, bool droppable, QLabel* &out) {
        auto *w = new QWidget();
        auto *l = new QVBoxLayout(w);
        auto *title_label = new QLabel(title);
        title_label->setAlignment(Qt::AlignCenter);
        l->addWidget(title_label);
        
        if(droppable) {
            auto *drop_label = new DropImageLabel();
            connect(drop_label, &DropImageLabel::fileDropped, this, &SpiegelMessApp::load_image_from_path);
            out = drop_label;
        } else {
            out = new QLabel();
        }
        
        out->setFixedSize(400, 400);
        out->setStyleSheet("border: 1px solid #ccc; background-color: #fafafa;");
        out->setAlignment(Qt::AlignCenter);
        l->addWidget(out);
        return w;
    }
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    SpiegelMessApp window;
    window.show();
    return app.exec();
}

#include "main.moc"
