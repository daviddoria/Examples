Imports System.Drawing.Printing 
Public Class DemoForm
    Inherits System.Windows.Forms.Form

#Region " Windows Form Designer generated code "

    Public Sub New()
        MyBase.New()

        'This call is required by the Windows Form Designer.
        InitializeComponent()

        'Add any initialization after the InitializeComponent() call

    End Sub

    'Form overrides dispose to clean up the component list.
    Protected Overloads Overrides Sub Dispose(ByVal disposing As Boolean)
        If disposing Then
            If Not (components Is Nothing) Then
                components.Dispose()
            End If
        End If
        MyBase.Dispose(disposing)
    End Sub

    'Required by the Windows Form Designer
    Private components As System.ComponentModel.IContainer

    'NOTE: The following procedure is required by the Windows Form Designer
    'It can be modified using the Windows Form Designer.  
    'Do not modify it using the code editor.
    Friend WithEvents TextToPrintTextBox As System.Windows.Forms.TextBox
    Friend WithEvents LoadTextFileButton As System.Windows.Forms.Button
    Friend WithEvents PrintButton As System.Windows.Forms.Button
    Friend WithEvents PrintDocument1 As System.Drawing.Printing.PrintDocument
    <System.Diagnostics.DebuggerStepThrough()> Private Sub InitializeComponent()
        Me.LoadTextFileButton = New System.Windows.Forms.Button()
        Me.PrintDocument1 = New System.Drawing.Printing.PrintDocument()
        Me.TextToPrintTextBox = New System.Windows.Forms.TextBox()
        Me.PrintButton = New System.Windows.Forms.Button()
        Me.SuspendLayout()
        '
        'LoadTextFileButton
        '
        Me.LoadTextFileButton.Location = New System.Drawing.Point(36, 24)
        Me.LoadTextFileButton.Name = "LoadTextFileButton"
        Me.LoadTextFileButton.Size = New System.Drawing.Size(120, 23)
        Me.LoadTextFileButton.TabIndex = 0
        Me.LoadTextFileButton.Text = "Load Text File"
        '
        'TextToPrintTextBox
        '
        Me.TextToPrintTextBox.Location = New System.Drawing.Point(36, 60)
        Me.TextToPrintTextBox.Multiline = True
        Me.TextToPrintTextBox.Name = "TextToPrintTextBox"
        Me.TextToPrintTextBox.ScrollBars = System.Windows.Forms.ScrollBars.Both
        Me.TextToPrintTextBox.Size = New System.Drawing.Size(348, 288)
        Me.TextToPrintTextBox.TabIndex = 1
        Me.TextToPrintTextBox.Text = ""
        '
        'PrintButton
        '
        Me.PrintButton.Location = New System.Drawing.Point(180, 24)
        Me.PrintButton.Name = "PrintButton"
        Me.PrintButton.Size = New System.Drawing.Size(120, 23)
        Me.PrintButton.TabIndex = 2
        Me.PrintButton.Text = "Print"
        '
        'DemoForm
        '
        Me.AutoScaleBaseSize = New System.Drawing.Size(5, 13)
        Me.BackColor = System.Drawing.SystemColors.ActiveCaptionText
        Me.ClientSize = New System.Drawing.Size(424, 374)
        Me.Controls.AddRange(New System.Windows.Forms.Control() {Me.PrintButton, Me.TextToPrintTextBox, Me.LoadTextFileButton})
        Me.Name = "DemoForm"
        Me.Text = "F00011 Print a Text File"
        Me.ResumeLayout(False)

    End Sub

#End Region


    Private Sub LoadTextFileButton_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles LoadTextFileButton.Click
        ' Declare a variable to hold the document path.
        ' Declare a variable of type String named documentPath.
        Dim documentPath As String = ""

        ' Declare, instantiate, and intialize a OpenFileDialog object
        ' to be used to pick a document.
        Dim openDocumentFileDialog As New OpenFileDialog()
        openDocumentFileDialog.InitialDirectory = "C:\"
        openDocumentFileDialog.RestoreDirectory = True
        openDocumentFileDialog.CheckFileExists = True
        openDocumentFileDialog.Filter = "Text files (*.txt)|*.txt"

        ' If openDocumentFileDialog OK button is clicked ....
        If openDocumentFileDialog.ShowDialog() = DialogResult.OK Then
            documentPath = openDocumentFileDialog.FileName
            Dim typeStreamReader As New System.IO.StreamReader(documentPath)
            Me.TextToPrintTextBox.Text = typeStreamReader.ReadToEnd
            typeStreamReader.Close()
        End If
    End Sub

    Private Sub PrintButton_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles PrintButton.Click
        AddHandler PrintDocument1.PrintPage, New System.Drawing.Printing.PrintPageEventHandler(AddressOf printPage)
        PrintDocument1.Print()
    End Sub

    Public Sub PrintPage(ByVal sender As Object, ByVal e As System.Drawing.Printing.PrintPageEventArgs)
        Static lastCharacterPrinted As Int32
        Dim printFont As New Font("Microsoft Sans Serif", 12)

        Dim PrintAreaHeight, PrintAreaWidth, LeftMargin, TopMargin As Int32
        With PrintDocument1.DefaultPageSettings
            PrintAreaHeight = .PaperSize.Height - .Margins.Top - .Margins.Bottom
            PrintAreaWidth = .PaperSize.Width - .Margins.Left - .Margins.Right
            LeftMargin = .Margins.Left
            TopMargin = .Margins.Top
        End With

        Dim lineCount As Integer = CInt(PrintAreaHeight / Font.Height)
        Dim printArea As New RectangleF(LeftMargin, TopMargin, PrintAreaWidth, PrintAreaHeight)

        
        Dim printFormat As New StringFormat(StringFormatFlags.LineLimit)

        Dim linesFittedToPage As Integer
        Dim charactersFittedToPage As Integer

        e.Graphics.MeasureString(Mid(Me.TextToPrintTextBox.Text, lastCharacterPrinted + 1), printFont, _
                    New SizeF(PrintAreaWidth, PrintAreaHeight), printFormat, _
                    charactersFittedToPage, linesFittedToPage)

        e.Graphics.DrawString(Mid(Me.TextToPrintTextBox.Text, lastCharacterPrinted + 1), printFont, _
            Brushes.Black, printArea, printFormat)

        lastCharacterPrinted += charactersFittedToPage


        If lastCharacterPrinted < Me.TextToPrintTextBox.Text.Length Then
            e.HasMorePages = True
        Else
            e.HasMorePages = False
            lastCharacterPrinted = 0
        End If
    End Sub


End Class
