from textual.widgets import DataTable
from logging_config import logger

class TopicTable(DataTable):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.add_columns("Topic", "Type")
        self.cursor_type = "row"
        self.current_topics = set()
        self.highlighted_row_key = None
        self.logger = logger

    def update_topics(self, topics):
        new_topic_names = {topic for topic, _ in topics}
        for topic, topic_type in topics:
            if topic not in self.current_topics:
                self.add_row(topic, str(topic_type), key=topic)
                self.current_topics.add(topic)
        for topic in list(self.current_topics):
            if topic not in new_topic_names:
                self.remove_row(topic)
                self.current_topics.remove(topic)

        coordinates = (0, 0)
        if self.highlighted_row_key is not None and self.highlighted_row_key in self.rows:
            row_index = self.get_row_index(self.highlighted_row_key)
            coordinates = (row_index, 0)
        elif self.row_count > 0:
            coordinates = (0, 0)
        self.logger.info(f"[TopicTable] Setting cursor coordinate to: {coordinates}")
        self.cursor_coordinate = coordinates
